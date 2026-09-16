/***************************************************************************************************
* @版    权：深圳拓邦股份有限公司-微电研发中心-家电部
* @文 件 名：user_motor.c
* @内容摘要：电机控制应用层：开环角度 + FOC 电流闭环
* @详细说明：电机控制应用层实现
*           1. ADC1 注入转换完成中断中以 20kHz 标称频率执行快速环
*              更新开环角度、采样三相电流、运行 FOC、更新三路 PWM 占空比
*           2. 速度环 PI 控制（1kHz），输出 Iq 电流参考值
*           3. 通过 MT6816 编码器获取转子电角度与电角速度
*           4. 启动流程（预定位）→ FOC 闭环运行（ADC偏置校准 + 功率PWM输出）
* @当前版本：V1.0
* @作    者：家电部-软件组
* @完成日期：
* @记    录：
* @修改记录：
* @修改日期：
* @版 本 号：
* @修 改 人：
***************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "user_motor.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_pwm_foc.h"
#include "bsp_mt6816ct_acd.h"
#include "user_motor_hall_vf.h"
#include "user_foc.h"
#include "user_foc_pid.h"
#include "user_smo.h"

/* ===== 临时堵转电感测试开关(堵转法测 Ls，测完移除) ===== */
#define USER_MOTOR_LR_TEST_ENABLE   (1u)      /* 0=关闭(恢复正常跑电机) */
#define USER_MOTOR_LR_TEST_DUTY1    (64u)     /* 第一档占空比偏移(Q10)，保持采样有效 */
#define USER_MOTOR_LR_TEST_DUTY2    (128u)    /* 第二档占空比偏移(Q10) */
#define USER_MOTOR_LR_TEST_SETTLE   (1u)      /* 施加电压后等待周期数 */
#define USER_MOTOR_LR_TEST_SPAN     (3u)      /* 斜率测量跨度(周期数，3=150us) */
#define USER_MOTOR_LR_TEST_ZERO     (150u)    /* 脉冲间零电压去磁周期数(≈7.5ms) */
#include <math.h>


/** @brief 电机启动状态机状态 */
typedef enum
{
    E_USR_MOTOR_STARTUP_ALIGN = 0,   /**< 预定位阶段 */
    E_USR_MOTOR_STARTUP_FOC = 1      /**< FOC 闭环阶段 */
} eUsrMotorStartupStateDef;

/** @brief 启动(预定位)阶段状态 */
typedef struct tUsrMotorStartupStateDef
{
    float f32AlignmentVoltage;       /**< 预定位斜坡升压电压(V) */
    float f32EncoderOffset;          /**< 编码器电气角度偏移(rad)，预定位完成后校准 */
    uint32_t u32Counter;             /**< 预定位计时计数(20kHz 周期数) */
    eUsrMotorStartupStateDef eState; /**< 启动状态机状态 */
} tUsrMotorStartupStateDef;

/** @brief 速度环状态 */
typedef struct tUsrMotorSpeedStateDef
{
    float f32Target;                /**< 目标转速(rpm)，由 UsrMotorSetSpeedTarget 设置 */
    float f32Ref;             /**< 速度环参考输入(rpm)，作为速度 PI 设定值 */
    float f32MechanicalRpm;         /**< 机械转速(rpm) */
    float f32ElecAngleSpeedFilter;  /**< 电角速度一阶低通滤波状态(rad/s) */
    float f32IqTarget;              /**< 速度环 PI 输出的 q 轴电流目标(A) */
    uint16_t u16PreEncoderRaw; /**< 上一次编码器原始角度(14 位计数)，用于差分测速 */
    uint8_t u8EncoderInitialized;   /**< 编码器历史角度是否已初始化标志 */
    tUserSpeedPidDef tPid;          /**< 速度环 PI 控制器 */
} tUsrMotorSpeedStateDef;

/** @brief 电流环状态 */
typedef struct tUsrMotorCurrentStateDef
{
    float f32IqRef;  /**< 斜坡处理后的 q 轴电流参考(A) */
    tFocInputDef tInput;   /**< FOC 电流环输入(三相电流、电角度、Id/Iq 参考) */
    tFocOutputDef tOutput; /**< FOC 电流环输出(三相 PWM 占空比) */
} tUsrMotorCurrentStateDef;

/** @brief 过流故障状态 */
typedef struct tUsrMotorFaultStateDef
{
    float f32Ia;                   /**< 过流故障时记录的 A 相电流(A) */
    float f32Ib;                   /**< 过流故障时记录的 B 相电流(A) */
    float f32Ic;                   /**< 过流故障时记录的 C 相电流(A) */
    uint8_t u8OverCurrentLatched;  /**< 过流故障锁存标志 */
    uint8_t u8OverCurrentCount;    /**< 过流去抖计数 */
} tUsrMotorFaultStateDef;

/* 快环所需状态集中放置于 CCM SRAM，按启动、速度、电流和故障职责分组。 */
typedef struct tUsrMotorControlStateDef
{
    tUsrMotorStartupStateDef tStartup; /**< 启动(预定位)阶段状态 */
    tUsrMotorSpeedStateDef tSpeed;     /**< 速度环状态 */
    tUsrMotorCurrentStateDef tCurrent; /**< 电流环状态 */
    tUsrMotorFaultStateDef tFault;     /**< 过流故障状态 */
} tUsrMotorControlStateDef;

/* 电机控制状态总结构体，置于 CCM SRAM 供 20kHz 快环低延迟访问 */
static tUsrMotorControlStateDef tMotor USER_MOTOR_CCMRAM;

/* Keil Logic Analyzer 中使用 g_tUserMotorScope.<成员名> 观察速度环。 */
volatile tUserMotorScopeDef g_tUserMotorScope;

/**
 * @brief   计算浮点数绝对值
 * @param[in] f32Value  输入值
 * @return  输入值的绝对值
 */
static float UsrMotorAbsFloat(float f32Value)
{
    return (f32Value >= 0.0f) ? f32Value : -f32Value;
}

/**
 * @brief   检查三相电流是否超过过流保护阈值
 * @param[in] f32Ia  A相电流(A)
 * @param[in] f32Ib  B相电流(A)
 * @param[in] f32Ic  C相电流(A)
 * @retval 1  任一相电流绝对值超过 USER_MOTOR_PHASE_CURRENT_LIMIT
 * @retval 0  三相电流均未超限
 */
static uint8_t UsrMotorIsPhaseCurrentOverLimit(float f32Ia, float f32Ib, float f32Ic)
{
    return (UsrMotorAbsFloat(f32Ia) > USER_MOTOR_PHASE_CURRENT_LIMIT) ||
           (UsrMotorAbsFloat(f32Ib) > USER_MOTOR_PHASE_CURRENT_LIMIT) ||
           (UsrMotorAbsFloat(f32Ic) > USER_MOTOR_PHASE_CURRENT_LIMIT);
}

/**
 * @brief   复位启动相关状态
 * @note    清零启动角度、Iq参考、预定位电压、速度反馈、启动计数和编码器历史，
 *          并将启动状态机置为预定位(ALIGN)
 */
static void UsrMotorResetStartup(void)
{
    tMotor.tCurrent.f32IqRef = 0.0f;       /* Iq参考清零 */
    tMotor.tStartup.f32AlignmentVoltage = 0.0f;  /* 开环电压清零 */
    tMotor.tStartup.u32Counter = 0u;             /* 启动计数清零 */
    tMotor.tStartup.eState = E_USR_MOTOR_STARTUP_ALIGN;   /* 状态置为预定位 */
    tMotor.tStartup.f32EncoderOffset = 0.0f;  /* 编码器偏移清零 */
    tMotor.tSpeed.u16PreEncoderRaw = 0u;    /* 上一次角度清零 */
    tMotor.tSpeed.u8EncoderInitialized = 0u;    /* 无历史采样 */
    tMotor.tSpeed.f32ElecAngleSpeedFilter = 0.0f;   /* 电角速度滤波状态清零 */
    tMotor.tSpeed.f32IqTarget = 0.0f;           /* 速度环Iq目标清零 */
}

/**
 * @brief   浮点数按固定步进斜坡逼近目标值
 * @param[in] f32Current  当前值
 * @param[in] f32Target   目标值
 * @param[in] f32Step     单次逼近步进(正值)
 * @return  斜坡后的当前值，越过目标时钳位到目标值
 */
static float UsrRampFloat(float f32Current, float f32Target, float f32Step)
{
    if (f32Current < f32Target)
    {
        f32Current += f32Step;                 /* 向上逼近 */
        if (f32Current > f32Target)
        {
            f32Current = f32Target;            /* 限制不超过目标 */
        }
    }
    else if (f32Current > f32Target)
    {
        f32Current -= f32Step;                 /* 向下逼近 */
        if (f32Current < f32Target)
        {
            f32Current = f32Target;            /* 限制不低于目标 */
        }
    }

    return f32Current;
}

/**
 * @brief   采样三相电流并更新过流保护状态
 * @retval 1  本次检测到电流超限
 * @retval 0  未超限
 * @note    连续超限达到去抖次数后置过流故障，记录故障电流，复位启动/FOC 状态并
 *          输出零电压；未超限时清零去抖计数
 */
static uint8_t UsrMotorCheckOverCurrent(void)
{
    tMotor.tCurrent.tInput.f32Ia = BspAdcGetIa();            /* 采样A相电流 */
    tMotor.tCurrent.tInput.f32Ib = BspAdcGetIb();            /* 采样B相电流 */
    tMotor.tCurrent.tInput.f32Ic = BspAdcGetIc();            /* 采样C相电流 */

    if (UsrMotorIsPhaseCurrentOverLimit(tMotor.tCurrent.tInput.f32Ia, tMotor.tCurrent.tInput.f32Ib, tMotor.tCurrent.tInput.f32Ic) != 0u)
    {
        tMotor.tFault.u8OverCurrentCount++;           /* 过流去抖计数累加 */
        if (tMotor.tFault.u8OverCurrentCount >= USER_MOTOR_OVC_DEBOUNCE_COUNT)
        {
            tMotor.tFault.f32Ia = tMotor.tCurrent.tInput.f32Ia;   /* 记录故障A相电流 */
            tMotor.tFault.f32Ib = tMotor.tCurrent.tInput.f32Ib;   /* 记录故障B相电流 */
            tMotor.tFault.f32Ic = tMotor.tCurrent.tInput.f32Ic;   /* 记录故障C相电流 */
            tMotor.tFault.u8OverCurrentLatched = 1u;       /* 置过流故障标志 */
            UsrMotorResetStartup();               /* 复位启动状态 */
            UsrFocReset();                        /* 复位FOC状态 */
            BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);   /* 输出零电压 */
        }
        return 1u;                                /* 返回过流 */
    }



    tMotor.tFault.u8OverCurrentCount = 0u;               /* 未超限清零去抖计数 */
    return 0u;
}

/**
 * @brief   编码器角度差回绕修正
 * @param[in] s32Delta  两次采样原始角度差(计数，未修正)
 * @return  修正回绕后的角度差(计数)
 * @note    编码器为循环计数(0~16383)，跨 0 点差分会产生接近一整圈的假差值；
 *          当差值超过半圈阈值时加减一整圈，还原两拍(1ms)内的真实位移；
 *          使用前提是单个速度环周期内转子机械位移小于半圈
 */
static int32_t UsrMotorWrapEncoderDelta(int32_t s32Delta)
{
    if (s32Delta > USER_MOTOR_ENC_WRAP_HALF_TURN)
    {
        s32Delta -= (int32_t)BSP_MT6816CT_ACTIVE;   /* 处理正向回绕 */
    }
    else if (s32Delta < -USER_MOTOR_ENC_WRAP_HALF_TURN)
    {
        s32Delta += (int32_t)BSP_MT6816CT_ACTIVE;   /* 处理负向回绕 */
    }

    return s32Delta;
}

/**
 * @brief   由快环最新编码器原始角度计算电角速度与机械转速(1kHz 调用)
 * @retval 1  速度计算完成
 * @note    编码器 SPI 读取已在 20kHz 快环(UsrMotorFastLoop)中完成，
 *          本函数仅在 1kHz 速度环中对快环存储的最新原始角度做 1ms 差分
 *          测速并一阶低通滤波，更新电角速度与机械转速。
 *          首次调用只记录历史不计算速度。
 */
static uint8_t UsrMotorUpdateSpeed(void)
{
    uint16_t EncoderRaw = BspMt6816CtGetRawCw();   /* 快环已读取的顺时针正方向角度 */
    int32_t s32Delta;
    float elecAngleSpeed;
    float alpha = USER_MOTOR_ENCODER_SPEED_FILTER_ALPHA;

    if (tMotor.tSpeed.u8EncoderInitialized == 0u)
    {
        tMotor.tSpeed.u8EncoderInitialized = 1u;
        tMotor.tSpeed.u16PreEncoderRaw = EncoderRaw;
        return 1u;
    }

    s32Delta = (int32_t)EncoderRaw - (int32_t)tMotor.tSpeed.u16PreEncoderRaw;   /* 顺时针坐标下的角度增量 */
    s32Delta = UsrMotorWrapEncoderDelta(s32Delta);   /* 修正 14 位角度回绕 */

    elecAngleSpeed = ((float)s32Delta * M_2PI * (float)MOTOR_POLE_PAIRS * (float)MOTOR_SPEED_LOOP_HZ)
                    / (float)BSP_MT6816CT_ACTIVE;

    tMotor.tSpeed.f32ElecAngleSpeedFilter += alpha *
                            (elecAngleSpeed - tMotor.tSpeed.f32ElecAngleSpeedFilter);

    tMotor.tSpeed.f32MechanicalRpm = (tMotor.tSpeed.f32ElecAngleSpeedFilter * 60.0f
                      / (M_2PI * (float)MOTOR_POLE_PAIRS));   /* 转机械 RPM */

    tMotor.tSpeed.u16PreEncoderRaw = EncoderRaw;   /* 更新上一次角度 */
    return 1u;
}

/**
 * @brief   校准编码器电气角度偏移
 * @note    以当前原始角度换算电角度作为偏移量保存，并将开环角度清零；
 *          在预定位完成后调用，使 FOC 角度与转子磁场对齐
 */
static void UsrMotorCalibrateEncoderOffset(void)
{
    tMotor.tStartup.f32EncoderOffset = BspMt6816CtToElecAngle(BspMt6816CtGetRawCw(),MOTOR_POLE_PAIRS,0.0f);
    tMotor.tSpeed.u8EncoderInitialized = 0u;   /* 强制下次测速重新初始化历史角度 */
}

/**
 * @brief   速度环 1kHz 周期处理(TIM7 中断调用)
 * @note    将速度目标斜坡到速度参考，再由编码器数据计算机械转速并做速度 PI，
 *          输出 q 轴电流目标
 */
void UsrMotorSpeedLoop(void)
{
    UsrSmoSlowUpdate();
    // tMotor.tSpeed.f32Ref = UsrRampFloat(tMotor.tSpeed.f32Ref,
    //                                                tMotor.tSpeed.f32Target,
    //                                                USER_MOTOR_SPEED_REF_STEP);
    UsrMotorUpdateSpeed();
    tMotor.tSpeed.f32IqTarget = UserSpeedPidCalculate(&tMotor.tSpeed.tPid,
                                                         tMotor.tSpeed.f32Ref,
                                                         tMotor.tSpeed.f32MechanicalRpm,
                                                         (float)MOTOR_SPEED_LOOP_HZ);   /* 速度环PI输出 */
}

/**
 * @brief   读取电位器 ADC 得到速度目标
 * @return  目标速度(rpm)，以 ADC 中点为零速
 */
float UsrMotorGetSpeedRef(void)
{
    uint16_t Raw = BspAdc2GetRaw(E_BSP_ADC2_POT);   /* 读取电位器ADC原始值 */
    float f32index = (float)(Raw - USER_MOTOR_ADC_MID_SCALE) / USER_MOTOR_ADC_MID_SCALE;
    return (f32index * USER_MOTOR_SPEED_REF_MAX);
}

/**
 * @brief   设置速度目标
 * @param[in] f32Target  速度目标(rpm)
 * @note    由系统任务按运行状态门控：RUNNING 时传电位器速度，否则传 0
 */
void UsrMotorSetSpeedTarget(float f32Target)
{
    tMotor.tSpeed.f32Target = f32Target;
}

/**
 * @brief   电机控制初始化
 * @note    清零控制状态结构体、复位启动状态，初始化 MT6816 编码器与 FOC，
 *          需在 main() 启动前调用
 */
void UsrMotorInit(void)
{
    memset(&tMotor, 0, sizeof(tMotor));          /* 清零控制状态结构体 */
    UserSpeedPidInit(&tMotor.tSpeed.tPid,
                     -0.003, -0.01,
                     USER_MOTOR_IQ_REF_MAX, -USER_MOTOR_IQ_REF_MAX,
                     USER_MOTOR_SPEED_ERROR_DEADBAND);   /* 速度环PI参数 */
    UsrMotorResetStartup();                      /* 复位启动状态 */
    BspMt6816CtInit();                        /* 初始化编码器 */
    UsrFocReset();                               /* 复位FOC状态 */
    UsrSmoInit();                                /* 初始化无感观测器影子运行状态 */
}

/**
 * @brief   启动电机(ADC 采样 + PWM 输出)
 * @retval HAL_OK      启动成功
 * @retval HAL_ERROR   启动失败(校准失败等)
 * @retval HAL_TIMEOUT 启动超时
 * @retval HAL_BUSY    忙
 * @note    依次启动 ADC1 注入采样、TIM1_CH4 触发与功率 PWM 输出并输出零电压；
 *          任一阶段失败会打印 RTT 日志并返回对应状态码，功率输出失败时执行
 *          BspPwmStop 停机
 */
HAL_StatusTypeDef UsrMotorStart(void)
{
    HAL_StatusTypeDef Status;

    Status = BspAdcStartInjected();              /* 启动ADC1注入采样 */
    if (Status != HAL_OK)
    {
        SEGGER_RTT_printf(0,
                          "MOTOR_START_FAIL,stage=ADC_INJECTED,status=%u\r\n",
                          (unsigned int)Status);
        return Status;
    }

    Status = BspPwmStartAdcTrigger();            /* 启动TIM1_CH4触发 */
    if (Status != HAL_OK)
    {
        SEGGER_RTT_printf(0,
                          "MOTOR_START_FAIL,stage=ADC_TRIGGER,status=%u\r\n",
                          (unsigned int)Status);
        return Status;
    }

    BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);       /* 输出零电压 */
    Status = BspPwmStartPowerOutputs();          /* 使能功率PWM输出 */
    if (Status != HAL_OK)
    {
        SEGGER_RTT_printf(0,
                          "MOTOR_START_FAIL,stage=POWER_PWM,status=%u\r\n",
                          (unsigned int)Status);
        BspPwmStop();
        return Status;
    }

    /* 偏置校准已由 BspAdcPreOffset 在启动前完成 */
#if (USER_MOTOR_LR_TEST_ENABLE != 0u)
    UsrMotorArmLockedRotorTest();            /* 堵转测 Ls：启动后自动执行一次 */
#endif
    SEGGER_RTT_WriteString(0, "Motor startup ready\r\n");
    return HAL_OK;
}

/**
 * @brief   判断是否允许功率输出
 * @retval 1  允许输出
 * @retval 0  禁止输出
 * @note    仅检查 ADC 电流偏置校准是否有效(BspAdcIsPreOffsetValid)，系统状态门控
 *          由速度目标实现
 */
static uint8_t UsrMotorIsOutputAllowed(void)
{
    return (uint8_t)((BspAdcIsPreOffsetValid() != 0u));
}


/**
 * @brief   停机序列
 * @note    复位 FOC 状态并输出零电压，用于速度参考低于停止阈值或过流故障时的停机
 */
static void UsrMotorStopSequence(void)
{
    // UsrMotorResetStartup();                  /* 复位启动状态 */
    UsrFocReset();                           /* 复位FOC状态 */
    BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);   /* 输出零电压 */
}

/**
 * @brief   启动预定位阶段处理
 * @note    以固定角度(0 rad)斜坡升压施加电压矢量，将转子拉到已知电角度位置；
 *          达到预定位时间后校准编码器偏移并切换到 FOC 闭环状态
 */
static void UsrMotorRunAlign(void)
{
    tMotor.tStartup.f32AlignmentVoltage = UsrRampFloat(tMotor.tStartup.f32AlignmentVoltage,   /* 斜坡升压 */
                                               MOTOR_ENCODER_ALIGN_VOLTAGE,
                                               MOTOR_ENCODER_ALIGN_STEP);
    UserMotorHallVfSetVoltageVector(0.0f, tMotor.tStartup.f32AlignmentVoltage);   /* 施加预定位电压矢量 */

    tMotor.tStartup.u32Counter++;              /* 预定位计数累加 */
    if (tMotor.tStartup.u32Counter >= USER_MOTOR_ENCODER_ALIGN_TICKS)   /* 达到预定位时间 */
    {
        UsrMotorCalibrateEncoderOffset();    /* 校准编码器偏移 */
        tMotor.tStartup.eState = E_USR_MOTOR_STARTUP_FOC;   /* 切换到FOC闭环 */
        tMotor.tStartup.f32AlignmentVoltage = 0.0f;        /* 开环电压清零 */
        UsrFocReset();
        UsrSmoReset();
    }
}

/**
 * @brief   FOC 闭环控制段
 * @note    编码器角度在 UsrMotorFastLoop 中以 20kHz 刷新，此处直接使用
 *          tMotor.tStartup.f32ElectricalAngle 实测电角度，无需插值。
 *          斜坡逼近速度环输出的 Iq 目标，运行电流环并更新三相 PWM 占空比。
 */
static void UsrMotorRunFoc(void)
{
    float elecAngle;
    float f32BusVoltage;
    tAlphaBetaCurrentDef tAlphaBetaCurrent;
    static float f32IdRef = 0.0f;

    /* 当前阶段仅影子运行：FOC 与速度环仍使用编码器反馈，SMO 不参与控制。 */
    elecAngle = BspMt6816CtToElecAngle(BspMt6816CtGetRawCw(),
                                       MOTOR_POLE_PAIRS,
                                       tMotor.tStartup.f32EncoderOffset);

    tMotor.tCurrent.f32IqRef = UsrRampFloat(tMotor.tCurrent.f32IqRef,
                                            tMotor.tSpeed.f32IqTarget,
                                            USER_MOTOR_IQ_REF_STEP);
    tMotor.tCurrent.tInput.f32IqRef = tMotor.tCurrent.f32IqRef;
    tMotor.tCurrent.tInput.f32Theta = elecAngle;
    tMotor.tCurrent.tInput.f32IdRef = f32IdRef;
    UsrFocCurrentLoop(&tMotor.tCurrent.tInput, &tMotor.tCurrent.tOutput);

    tAlphaBetaCurrent = UsrFocGetAlphaBetaCurrent();
    f32BusVoltage = BspAdc2GetVoltage(E_BSP_ADC2_VBUS) * USER_MOTOR_VBUS_DIVIDER_RATIO;

    UsrSmoSetEncoderReference(elecAngle, tMotor.tSpeed.f32ElecAngleSpeedFilter);
    UsrSmoFastUpdate(tAlphaBetaCurrent.f32Alpha,
                     tAlphaBetaCurrent.f32Beta,
                     &tMotor.tCurrent.tOutput.tDuty,
                     f32BusVoltage);

    BspPwmSetDutyQ10(tMotor.tCurrent.tOutput.tDuty.u16A,
                     tMotor.tCurrent.tOutput.tDuty.u16B,
                     tMotor.tCurrent.tOutput.tDuty.u16C);
}

/* ===== 临时堵转电感测试(堵转法测 Ls，测完移除，开关见文件头) =====
   电机启动后自动执行一次：分两档电压施加 A+/B- 固定矢量，各取两个电流点算斜率，
   堵转时 BEMF=0，电压方程 v = Rs·i + Ls·di/dt 消去 Rs 得 Ls，经 RTT 输出。
   使用前需堵转转子；测完输出零电压，随后按速度参考正常控制(参考为 0 则保持静止)。 */

typedef struct tUsrLrTestDef
{
    uint8_t u8State;
    uint8_t u8Ticks;
    float f32BusVoltage;
    float f32IStart;
    float f32Slope1;
    float f32Slope2;
    float f32Ls;
} tUsrLrTestDef;

static tUsrLrTestDef tLrTest;

#if (USER_MOTOR_LR_TEST_ENABLE != 0u)
/**
 * @brief   堵转电感测试状态机(20kHz 快环调用)
 * @note    每档电压取两个电流点计算 di/dt，消去 Rs 后由 v=Ls·di/dt 求 Ls
 */
static void UsrMotorRunLockedRotorTest(void)
{
    const float f32SpanSec = (float)USER_MOTOR_LR_TEST_SPAN / (float)USER_MOTOR_FAST_LOOP_HZ;

    switch (tLrTest.u8State)
    {
        case 1u:   /* 初始零电压去磁：让电流归零，保证初始斜率 R·i≈0 */
            BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
            tLrTest.f32BusVoltage = BspAdc2GetVoltage(E_BSP_ADC2_VBUS) * USER_MOTOR_VBUS_DIVIDER_RATIO;   /* 零负载下刷新母线电压 */
            tLrTest.u8Ticks++;
            if (tLrTest.u8Ticks >= USER_MOTOR_LR_TEST_ZERO)
            {
                tLrTest.u8Ticks = 0u;
                tLrTest.u8State = 2u;
            }
            break;
        case 2u:   /* 第一档电压，取初始斜率 */
            BspPwmSetDutyQ10((uint16_t)(512u + USER_MOTOR_LR_TEST_DUTY1),
                             (uint16_t)(512u - USER_MOTOR_LR_TEST_DUTY1), 512u);
            tLrTest.u8Ticks++;
            if (tLrTest.u8Ticks == USER_MOTOR_LR_TEST_SETTLE)
            {
                tLrTest.f32IStart = BspAdcGetIa();
            }
            else if (tLrTest.u8Ticks == (uint8_t)(USER_MOTOR_LR_TEST_SETTLE + USER_MOTOR_LR_TEST_SPAN))
            {
                tLrTest.f32Slope1 = (BspAdcGetIa() - tLrTest.f32IStart) / f32SpanSec;
                tLrTest.u8Ticks = 0u;
                tLrTest.u8State = 3u;
            }
            break;
        case 3u:   /* 中间零电压去磁 */
            BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
            tLrTest.f32BusVoltage = BspAdc2GetVoltage(E_BSP_ADC2_VBUS) * USER_MOTOR_VBUS_DIVIDER_RATIO;   /* 零负载下刷新母线电压 */
            tLrTest.u8Ticks++;
            if (tLrTest.u8Ticks >= USER_MOTOR_LR_TEST_ZERO)
            {
                tLrTest.u8Ticks = 0u;
                tLrTest.u8State = 4u;
            }
            break;
        case 4u:   /* 第二档电压，取初始斜率 */
            BspPwmSetDutyQ10((uint16_t)(512u + USER_MOTOR_LR_TEST_DUTY2),
                             (uint16_t)(512u - USER_MOTOR_LR_TEST_DUTY2), 512u);
            tLrTest.u8Ticks++;
            if (tLrTest.u8Ticks == USER_MOTOR_LR_TEST_SETTLE)
            {
                tLrTest.f32IStart = BspAdcGetIa();
            }
            else if (tLrTest.u8Ticks == (uint8_t)(USER_MOTOR_LR_TEST_SETTLE + USER_MOTOR_LR_TEST_SPAN))
            {
                tLrTest.f32Slope2 = (BspAdcGetIa() - tLrTest.f32IStart) / f32SpanSec;
                tLrTest.u8State = 5u;
            }
            break;
        case 5u:   /* 计算并 RTT 输出 */
        {
            float f32V1;
            float f32V2;
            float f32Ls1;
            float f32Ls2;
            float f32LsDiff;

            tLrTest.f32BusVoltage = BspAdc2GetVoltage(E_BSP_ADC2_VBUS) * USER_MOTOR_VBUS_DIVIDER_RATIO;   /* 零负载下刷新母线电压 */
            f32V1 = 2.0f * (float)USER_MOTOR_LR_TEST_DUTY1 /
                          (float)USER_FOC_DUTY_Q10_MAX * tLrTest.f32BusVoltage;
            f32V2 = 2.0f * (float)USER_MOTOR_LR_TEST_DUTY2 /
                          (float)USER_FOC_DUTY_Q10_MAX * tLrTest.f32BusVoltage;
            f32Ls1 = f32V1 / (2.0f * tLrTest.f32Slope1);   /* 初始斜率 R·i≈0，单档独立可算 */
            f32Ls2 = f32V2 / (2.0f * tLrTest.f32Slope2);
            f32LsDiff = (f32V2 - f32V1) / (2.0f * (tLrTest.f32Slope2 - tLrTest.f32Slope1));

            tLrTest.f32Ls = (f32LsDiff > 0.0f) ? f32LsDiff : ((f32Ls1 + f32Ls2) * 0.5f);
            if (tLrTest.f32Ls < 0.0f)
            {
                tLrTest.f32Ls = 0.0f;
            }
            SEGGER_RTT_printf(0, "LR_TEST d1=%dA/s d2=%dA/s Ls1=%duH Ls2=%duH Ls=%duH\r\n",
                              (int)tLrTest.f32Slope1, (int)tLrTest.f32Slope2,
                              (int)(f32Ls1 * 1000000.0f), (int)(f32Ls2 * 1000000.0f),
                              (int)(tLrTest.f32Ls * 1000000.0f));
            BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
            tLrTest.u8State = 0u;   /* 结束 */
            break;
        }
        default:
            break;
    }
}

/**
 * @brief   挂载堵转电感测试(启动后自动执行一次)
 * @note    需在堵转转子后启动电机；结果经 RTT 输出
 */
void UsrMotorArmLockedRotorTest(void)
{
    (void)BspAdc2UpdateAll();   /* 立即采样一次 ADC2(含 VBUS)，避免启动瞬间系统任务尚未采样 */
    tLrTest.f32BusVoltage = BspAdc2GetVoltage(E_BSP_ADC2_VBUS) * USER_MOTOR_VBUS_DIVIDER_RATIO;
    tLrTest.u8State = 1u;
}
#endif

/**
 * @brief   电机快速环 20kHz 中断服务(ADC 注入转换完成中断调用)
 * @note    流程：输出允许判断 → 采样有效性检查 → 过流保护 →
 *          编码器 SPI 读取(20kHz新鲜角度) → 速度参考/过流停机判断 →
 *          按启动状态机执行预定位或 FOC 闭环
 */
USER_MOTOR_FAST_CODE void UsrMotorFastLoop(void)
{

    if (UsrMotorIsOutputAllowed() == 0u)     /* 偏置未校准或输出未使能 */
    {
        BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
        return;
    }

    if (BspAdcIsSampleValid() == 0u)
    {
        return;                              /* 本次采样无效则跳过 */
    }

    if (UsrMotorCheckOverCurrent() != 0u)
    {
        return;                              /* 过流时停止输出 */
    }

#if (USER_MOTOR_LR_TEST_ENABLE != 0u)
    if (tLrTest.u8State != 0u)               /* 堵转测试：输出固定电压矢量并抓电流 */
    {
        UsrMotorRunLockedRotorTest();
        return;
    }
#endif

    /* 20kHz 读取编码器，获得新鲜电角度，无需 PLL 插值 */
    if (BspMt6816CtUpdate() != HAL_OK)
    {
        return;                              /* SPI 读取失败，跳过本周期 */
    }

    if ((tMotor.tSpeed.f32Ref < USER_MOTOR_SPEED_STOP_THRESHOLD) &&
        (tMotor.tSpeed.f32Ref > -USER_MOTOR_SPEED_STOP_THRESHOLD))   /* 速度目标接近零则停机 */
    {
        tMotor.tFault.u8OverCurrentLatched = 0u;
        return;
    }
    
    if (tMotor.tFault.u8OverCurrentLatched != 0u)     /* 过流故障则停机 */
    {
        UsrMotorStopSequence();
        return;
    }

    switch (tMotor.tStartup.eState)
    {
        case E_USR_MOTOR_STARTUP_ALIGN:                /* 预定位阶段 */
            UsrMotorRunAlign();
            break;

        case E_USR_MOTOR_STARTUP_FOC:                  /* FOC闭环控制 */
            UsrMotorRunFoc();
            break;

        default:                                       /* 未知状态保护 */
            BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
            break;
    }
}

/**
 * @brief   获取当前启动状态
 * @retval 0  预定位(ALIGN)
 * @retval 1  FOC 闭环
 */
uint8_t UsrMotorGetStartupMode(void)
{
    return (uint8_t)tMotor.tStartup.eState;
}




/**
 * @brief   查询过流故障状态
 * @retval 1  已触发过流故障
 * @retval 0  正常
 */
uint8_t UsrMotorIsOverCurrentFault(void)
{
    return tMotor.tFault.u8OverCurrentLatched;
}

/**
 * @brief   获取过流故障时的 A 相电流
 * @return  故障 A 相电流(A)
 */
float UsrMotorGetFaultIa(void)
{
    return tMotor.tFault.f32Ia;
}

/**
 * @brief   获取过流故障时的 B 相电流
 * @return  故障 B 相电流(A)
 */
float UsrMotorGetFaultIb(void)
{
    return tMotor.tFault.f32Ib;
}

/**
 * @brief   获取过流故障时的 C 相电流
 * @return  故障 C 相电流(A)
 */
float UsrMotorGetFaultIc(void)
{
    return tMotor.tFault.f32Ic;
}



float UsrMotorGetSpeed(void)
{
    return tMotor.tSpeed.f32MechanicalRpm;
}
