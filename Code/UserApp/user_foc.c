/***************************************************************************************************
* @版    权：深圳拓邦股份有限公司-微电研发中心-家电部
* @文 件 名：user_foc.c
* @内容摘要：FOC 电流环：Clarke/Park 变换 + PI 调节 + SVPWM 输出
* @详细说明：FOC 电流环定点实现
*           1. 三相电流采样码值经共模消除、Clarke 变换与 Park 变换得到 d/q 轴电流
*           2. d/q 轴电流分别由 PI 调节器闭环，输出 d/q 轴电压指令
*           3. SVPWM 将电压指令转换为三相占空比并做调制限幅与滤波
*           4. 全定点运算（Q16 增益 / Q10 角度 / Q15 滤波），20kHz 电流环调用
* @当前版本：V1.0
* @作    者：家电部-软件组
* @完成日期：
* @记    录：
* @修改记录：
* @修改日期：
* @版 本 号：
* @修 改 人：
***************************************************************************************************/

#include "user_foc.h"
#include "user_foc_math.h"
#include "user_foc_pid.h"
#include <string.h>




/* 三相电流经坐标变换后的定点码值 */
typedef struct tFocCurrentCodeDef
{
    int32_t s32A;         /* A相电流码值(去共模后) */
    int32_t s32B;         /* B相电流码值(去共模后) */
    int32_t s32C;         /* C相电流码值(去共模后) */
    int32_t s32Alpha;     /* α轴电流码值 */
    int32_t s32Beta;      /* β轴电流码值 */
    int32_t s32D;         /* d轴电流码值(滤波后) */
    int32_t s32Q;         /* q轴电流码值(滤波后) */
} tFocCurrentCodeDef;

/* FOC 内部状态结构体 */
typedef struct tFocFilterStateDef
{
    int64_t s64CurrentD;       /* d轴电流滤波状态(Q15) */
    int64_t s64CurrentQ;       /* q轴电流滤波状态(Q15) */
    int64_t s64DutyA;          /* A相占空比滤波状态(Q15) */
    int64_t s64DutyB;          /* B相占空比滤波状态(Q15) */
    int64_t s64DutyC;          /* C相占空比滤波状态(Q15) */
} tFocFilterStateDef;

typedef struct tFocReferenceDef
{
    float f32Id;
    float f32Iq;
} tFocReferenceDef;

typedef struct tFocContextDef
{
    tUserFocPidDef tPidD;        /* d轴电流PI调节器 */
    tUserFocPidDef tPidQ;        /* q轴电流PI调节器 */
    tFocCurrentCodeDef tCurrent; /* 当前电流码值 */
    tThreePhaseDutyDef tDuty;    /* 三相占空比输出 */
    tFocFilterStateDef tFilter;  /* 电流及占空比滤波状态 */
    tFocReferenceDef tReference; /* 最近一次d/q轴电流参考 */
} tFocContextDef;

typedef struct tFocTuningDef
{
    int32_t s32KpQ16;
    int32_t s32KiQ16;
} tFocTuningDef;

static tFocContextDef tFoc;

/* 调参值独立于运行状态，保持 UsrFocReset 后按键调参行为不变。 */
static tFocTuningDef tFocTuning =
{
    USER_FOC_PI_KP_Q16,
    USER_FOC_PI_KI_Q16
};

/* Keil Logic Analyzer 中使用 g_tUserFocScope.<成员名> 观察电流环。 */
volatile tUserFocScopeDef g_tUserFocScope;

/**
 * @brief   电流值(A)换算为电流码值(LSB)
 * @param[in] f32Current  电流值(A)
 * @return  换算后的电流码值(LSB)，按 USER_FOC_CURRENT_AMPS_PER_CODE 换算并四舍五入取整
 */
static int32_t UsrFocFloat2CurrCode(float f32Current)
{
    float f32Code = f32Current / USER_FOC_CURRENT_AMPS_PER_CODE;
    return (f32Code >= 0.0f) ? (int32_t)(f32Code + 0.5f) : (int32_t)(f32Code - 0.5f);
}

/**
 * @brief   电流码值(LSB)换算为电流值(A)
 * @param[in] s32Code  电流码值(LSB)
 * @return  换算后的电流值(A)
 */
static float UsrFocCurrCode2Float(int32_t s32Code)
{
    return (float)s32Code * USER_FOC_CURRENT_AMPS_PER_CODE;
}

/**
 * @brief   弧度角转换为 0.001rad 定点角度
 * @param[in] f32Angle  角度(rad)
 * @return  定点角度(0~6283，对应 0~2π)
 * @note    输入先规整到 [0, 2π)，再乘以 1000 并四舍五入，最后归一化到 0~6283，供查表函数使用
 */
static uint16_t UsrFocFloatToAngle1000(float f32Angle)
{
    while (f32Angle >= M_2PI) { f32Angle -= M_2PI; }   /* 规整到[0,2π) */
    while (f32Angle < 0.0f) { f32Angle += M_2PI; }
    return _normalizeAngle((int32_t)(f32Angle * 1000.0f + 0.5f));
}

/**
 * @brief   一阶低通滤波(Q15 定点)
 * @param[in,out] ps64StateQ15  滤波状态指针(Q15)，就地更新
 * @param[in]     s32Input      输入值(Q0)
 * @param[in]     s32AlphaQ15   滤波系数(Q15)，越大跟随越快
 * @return  滤波后的输出值(Q0)
 * @note    公式：输出 = 上次输出 + α×(输入 − 上次输出)，用于电流采样与占空比平滑
 */
static int32_t UsrFocFilterQ15(int64_t *ps64StateQ15,
                               int32_t s32Input,
                               int32_t s32AlphaQ15)
{
    int64_t s64TargetQ15 = (int64_t)s32Input << USER_FOC_FILTER_Q_SHIFT;
    int64_t s64DeltaQ15 = s64TargetQ15 - *ps64StateQ15;

    *ps64StateQ15 += (s64DeltaQ15 * s32AlphaQ15) >> USER_FOC_FILTER_Q_SHIFT;
    return (int32_t)(*ps64StateQ15 >> USER_FOC_FILTER_Q_SHIFT);
}

static uint16_t UsrFocLimitDutyQ10(int32_t s32Duty)
{
    if (s32Duty < 0)
    {
        return 0u;
    }
    if (s32Duty > (int32_t)USER_FOC_DUTY_Q10_MAX)
    {
        return USER_FOC_DUTY_Q10_MAX;
    }
    return (uint16_t)s32Duty;
}

/**
 * @brief   三相电流坐标变换(Clarke + Park)
 * @param[in] s32Ia     A相电流码值(LSB)
 * @param[in] s32Ib     B相电流码值(LSB)
 * @param[in] s32Ic     C相电流码值(LSB)
 * @param[in] u16Angle  电角度(0.001rad 定点)
 * @note    先消除三相共模分量，再经 Clarke 变换得到 α/β 轴电流，按电角度做 Park
 *          变换得到 d/q 轴电流并做一阶低通滤波，结果写入全局变量 tFoc.tCurrent
 */
static void UsrFocTransformCurrent(int32_t s32Ia, int32_t s32Ib, int32_t s32Ic, uint16_t u16Angle)
{
    int32_t s32CommonMode = (s32Ia + s32Ib + s32Ic) / 3;   /* 三相共模分量 */
    int32_t s32Sin = _sin_q10(u16Angle);   /* 电角度正弦(Q10) */
    int32_t s32Cos = _cos_q10(u16Angle);   /* 电角度余弦(Q10) */
    int32_t s32RawD;
    int32_t s32RawQ;

    tFoc.tCurrent.s32A = s32Ia - s32CommonMode;   /* 去除共模后的A相电流 */
    tFoc.tCurrent.s32B = s32Ib - s32CommonMode;
    tFoc.tCurrent.s32C = s32Ic - s32CommonMode;
    tFoc.tCurrent.s32Alpha = tFoc.tCurrent.s32A;   /* Clarke：α轴电流 */
    tFoc.tCurrent.s32Beta = FOC_MulQ10(tFoc.tCurrent.s32B - tFoc.tCurrent.s32C, ONE_OVER_SQRT3_Q10);   /* Clarke：β轴电流 = (B-C)/√3 */
    
    /* Park 变换：d = α·cos(θ) + β·sin(θ)，q = -α·sin(θ) + β·cos(θ) */
    s32RawD = FOC_MulQ10(tFoc.tCurrent.s32Alpha, s32Cos) + FOC_MulQ10(tFoc.tCurrent.s32Beta, s32Sin);   /* d轴电流 */
    s32RawQ = FOC_MulQ10(-tFoc.tCurrent.s32Alpha, s32Sin) + FOC_MulQ10(tFoc.tCurrent.s32Beta, s32Cos);   /* q轴电流 */
    tFoc.tCurrent.s32D = UsrFocFilterQ15(&tFoc.tFilter.s64CurrentD, s32RawD, USER_FOC_CURRENT_FILTER_ALPHA_Q15);   /* d轴电流滤波 */
    tFoc.tCurrent.s32Q = UsrFocFilterQ15(&tFoc.tFilter.s64CurrentQ, s32RawQ, USER_FOC_CURRENT_FILTER_ALPHA_Q15);   /* q轴电流滤波 */
}

/**
 * @brief   空间矢量调制 SVPWM，由 d/q 电压指令生成三相占空比
 * @param[in] s32Vd     d轴电压指令(Q10)
 * @param[in] s32Vq     q轴电压指令(Q10)
 * @param[in] u16Angle  电角度(0.001rad 定点)
 * @return  三相占空比结构体(滤波后，Q10)
 * @note    合成矢量幅值受 USER_FOC_MODULATION_LIMIT_Q10 限幅；按电角度确定扇区并
 *          计算相邻矢量作用时间 T1/T2 与零矢量时间 T0，零矢量均分到周期两端
 *          (中心对齐)，输出占空比再经一阶低通滤波
 */
static tThreePhaseDutyDef UsrFocSvpwm(int32_t s32Vd, int32_t s32Vq, uint16_t u16Angle)
{
    tThreePhaseDutyDef Result;
    int32_t s32Magnitude = 0;   /* 电压矢量幅值(Q10) */
    int32_t s32T1;              /* 相邻矢量1作用时间 */
    int32_t s32T2;              /* 相邻矢量2作用时间 */
    int32_t s32T0;              /* 零矢量作用时间 */
    int32_t s32DutyA;
    int32_t s32DutyB;
    int32_t s32DutyC;
    uint16_t u16Theta;
    uint8_t u8Sector;

    if ((s32Vd != 0) || (s32Vq != 0))
    {
        int64_t s64MagnitudeSquared = (int64_t)s32Vd * s32Vd + (int64_t)s32Vq * s32Vq;   /* 幅值平方 */
        if (s64MagnitudeSquared > 0x7FFFFFFFLL) { s64MagnitudeSquared = 0x7FFFFFFFLL; }   /* 防溢出 */
        s32Magnitude = _sqrt_fast((int32_t)s64MagnitudeSquared);   /* 开方得幅值 */
        u16Angle = _normalizeAngle((int32_t)u16Angle + fast_atan2_int(s32Vq, s32Vd));   /* 合成矢量角度 */
    }
    if (s32Magnitude > USER_FOC_MODULATION_LIMIT_Q10) { s32Magnitude = USER_FOC_MODULATION_LIMIT_Q10; }   /* 调制比限幅 */

    u8Sector = (uint8_t)(((uint32_t)u16Angle * 6u) / (uint32_t)_2PI_1000);   /* 扇区判断 */
    if (u8Sector > 5u) { u8Sector = 5u; }
    u16Theta = (uint16_t)(u16Angle - ((uint16_t)u8Sector * _PI_3_1000));   /* 扇区内角度 */
    s32T1 = FOC_MulQ10(FOC_MulQ10(SQRT3_Q10,
                                   _sin_q10((uint16_t)(_PI_3_1000 - u16Theta))),
                           s32Magnitude);   /* 相邻矢量1作用时间 */
    s32T2 = FOC_MulQ10(FOC_MulQ10(SQRT3_Q10, _sin_q10(u16Theta)), s32Magnitude);   /* 相邻矢量2作用时间 */
    s32T0 = FOC_Q10_ONE - s32T1 - s32T2;   /* 零矢量作用时间 */

    /* 按扇区分配三相占空比（中心对齐，零矢量均分到两端） */
    switch (u8Sector)
    {
        case 0u: s32DutyA = s32T1 + s32T2 + (s32T0 >> 1); s32DutyB = s32T2 + (s32T0 >> 1); s32DutyC = s32T0 >> 1; break;
        case 1u: s32DutyA = s32T1 + (s32T0 >> 1); s32DutyB = s32T1 + s32T2 + (s32T0 >> 1); s32DutyC = s32T0 >> 1; break;
        case 2u: s32DutyA = s32T0 >> 1; s32DutyB = s32T1 + s32T2 + (s32T0 >> 1); s32DutyC = s32T2 + (s32T0 >> 1); break;
        case 3u: s32DutyA = s32T0 >> 1; s32DutyB = s32T1 + (s32T0 >> 1); s32DutyC = s32T1 + s32T2 + (s32T0 >> 1); break;
        case 4u: s32DutyA = s32T2 + (s32T0 >> 1); s32DutyB = s32T0 >> 1; s32DutyC = s32T1 + s32T2 + (s32T0 >> 1); break;
        default: s32DutyA = s32T1 + s32T2 + (s32T0 >> 1); s32DutyB = s32T0 >> 1; s32DutyC = s32T1 + (s32T0 >> 1); break;
    }

    s32DutyA = UsrFocFilterQ15(&tFoc.tFilter.s64DutyA, s32DutyA, USER_FOC_DUTY_FILTER_ALPHA_Q15);   /* A相占空比滤波 */
    s32DutyB = UsrFocFilterQ15(&tFoc.tFilter.s64DutyB, s32DutyB, USER_FOC_DUTY_FILTER_ALPHA_Q15);
    s32DutyC = UsrFocFilterQ15(&tFoc.tFilter.s64DutyC, s32DutyC, USER_FOC_DUTY_FILTER_ALPHA_Q15);
    Result.u16A = UsrFocLimitDutyQ10(s32DutyA);
    Result.u16B = UsrFocLimitDutyQ10(s32DutyB);
    Result.u16C = UsrFocLimitDutyQ10(s32DutyC);
    return Result;
}

/**
 * @brief   复位 FOC 状态
 * @note    清零全部内部状态并重新初始化 d/q 轴 PI 调节器；占空比滤波状态与输出
 *          置 50% 中点，防止复位瞬间占空比跳变
 */
void UsrFocReset(void)
{
    memset(&tFoc, 0, sizeof(tFoc));   /* 清零FOC上下文 */
    UserFocPidInit(&tFoc.tPidD, -14800, -150,
                   USER_FOC_PI_KD_Q16, USER_FOC_PI_OUTPUT_MAX, USER_FOC_PI_OUTPUT_MIN);   /* 初始化d轴PI */

    UserFocPidInit(&tFoc.tPidQ, -14800, -150,
                   USER_FOC_PI_KD_Q16, USER_FOC_PI_OUTPUT_MAX, USER_FOC_PI_OUTPUT_MIN);   /* 初始化q轴PI */



    tFoc.tFilter.s64DutyA = (int64_t)(FOC_Q10_ONE / 2) << USER_FOC_FILTER_Q_SHIFT;   /* 占空比滤波置中点 */
    tFoc.tFilter.s64DutyB = (int64_t)(FOC_Q10_ONE / 2) << USER_FOC_FILTER_Q_SHIFT;
    tFoc.tFilter.s64DutyC = (int64_t)(FOC_Q10_ONE / 2) << USER_FOC_FILTER_Q_SHIFT;
    tFoc.tDuty.u16A = FOC_Q10_ONE / 2;   /* 输出占空比置50% */
    tFoc.tDuty.u16B = FOC_Q10_ONE / 2;
    tFoc.tDuty.u16C = FOC_Q10_ONE / 2;
}

/**
 * @brief   FOC 电流环主函数
 * @param[in]  ptInput   电流环输入(三相电流、电角度、d/q 电流参考)，不允许为 NULL
 * @param[out] ptOutput  电流环输出(三相占空比、d/q 电流)，可为 NULL
 * @note    依次完成坐标变换、d/q 轴电流 PI 闭环与 SVPWM 调制，并同步更新 scope 观测量
 */
    int32_t s32IdRefCode;
    int32_t s32IqRefCode;
    int32_t s32Vd;
    int32_t s32Vq;
void UsrFocCurrentLoop(const tFocInputDef *ptInput, tFocOutputDef *ptOutput)
{
    uint16_t u16Angle;


    if (ptInput == NULL)
    {
        return;
    }

    // tFoc.tReference.f32Id = ptInput->f32IdRef;
    // tFoc.tReference.f32Iq = ptInput->f32IqRef;
    s32IdRefCode = UsrFocFloat2CurrCode(ptInput->f32IdRef);
    s32IqRefCode = UsrFocFloat2CurrCode(ptInput->f32IqRef);

    u16Angle = UsrFocFloatToAngle1000(ptInput->f32Theta);
    UsrFocTransformCurrent(UsrFocFloat2CurrCode(ptInput->f32Ia),
                           UsrFocFloat2CurrCode(ptInput->f32Ib),
                           UsrFocFloat2CurrCode(ptInput->f32Ic),
                           u16Angle);

    s32Vd = UserFocPidCalculate(&tFoc.tPidD, s32IdRefCode, tFoc.tCurrent.s32D);
    s32Vq = UserFocPidCalculate(&tFoc.tPidQ, s32IqRefCode, tFoc.tCurrent.s32Q);
    tFoc.tDuty = UsrFocSvpwm(s32Vd, s32Vq, u16Angle);

    if (ptOutput != NULL)
    {
        ptOutput->tDuty = tFoc.tDuty;
        ptOutput->tCurrent.f32D = UsrFocCurrCode2Float(tFoc.tCurrent.s32D);
        ptOutput->tCurrent.f32Q = UsrFocCurrCode2Float(tFoc.tCurrent.s32Q);
    }
}

/**
 * @brief   获取当前 d/q 轴电流
 * @return  d/q 电流结构体(A)
 * @note    电流取自滤波后的 d/q 电流码值换算为安培
 */
tDqCurrentDef UsrFocGetDqCurrent(void)
{
    tDqCurrentDef Current;
    Current.f32D = UsrFocCurrCode2Float(tFoc.tCurrent.s32D);
    Current.f32Q = UsrFocCurrCode2Float(tFoc.tCurrent.s32Q);
    return Current;
}
