/**
 * @file    user_motor.c
 * @brief   电机控制应用层实现 — 开环角度 + FOC 电流闭环
 *******************************************************************************
 * @note    ADC1 注入转换完成中断中以 20kHz 标称频率执行快速环：
 *          更新开环角度、采样三相电流、运行 FOC、更新三相 PWM。
 *******************************************************************************
 */

#include "user_motor.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "bsp_hall.h"
#include "user_foc.h"
#include <math.h>

#define USER_MOTOR_PI              (3.14159265f)
#define USER_MOTOR_TWO_PI          (2.0f * USER_MOTOR_PI)
#define USER_MOTOR_IQ_REF_MAX      (0.5f)
#define USER_MOTOR_IQ_REF_STEP     (0.002f)
#define USER_MOTOR_FAST_LOOP_HZ     (20000u)
#define USER_MOTOR_ADC_FULL_SCALE  (4095.0f)
#define USER_MOTOR_IQ_ADC_DEADZONE (300u)
#define USER_MOTOR_THETA_STEP_INIT (0.001f)
#define USER_MOTOR_THETA_STEP_MAX  (0.006f)
#define USER_MOTOR_THETA_STEP_INC  (0.0000004f)
#define USER_MOTOR_IQ_STOP_THRESHOLD (0.02f)
#define USER_MOTOR_PHASE_CURRENT_LIMIT (10.0f)
#define USER_MOTOR_ALIGN_TIME_MS    (800u)
#define USER_MOTOR_ALIGN_TICKS      ((USER_MOTOR_FAST_LOOP_HZ * USER_MOTOR_ALIGN_TIME_MS) / 1000u)
#define USER_MOTOR_ALIGN_ID_REF     (0.4f)
#define USER_MOTOR_OFFSET_TIMEOUT_MS (500u)
#define USER_MOTOR_OVC_DEBOUNCE_COUNT (3u)

/* Temporary diagnostic modes. Keep only one active while checking phase wiring. */
#define USER_MOTOR_DEBUG_ADC_ONLY           (0u)      /* 1: keep CH1/CH2/CH3 and CH1N/CH2N/CH3N disabled; CH4 still triggers ADC. */
#define USER_MOTOR_DEBUG_ZERO_VECTOR_PWM     (0u)      /* 1: enable power PWM at 50%/50%/50% for sampling-point verification. */
#define USER_MOTOR_DEBUG_FIXED_VECTOR       (0u)
#define USER_MOTOR_DEBUG_FIXED_PHASE        (2u)      /* 0=A+, 1=B+, 2=C+ */
#define USER_MOTOR_DEBUG_FIXED_VOLTAGE      (1.5f)    /* Same percent-scale unit as BspPwmSetVoltageAbc. */
#define USER_MOTOR_DEBUG_OPEN_VOLTAGE       (0u)
#define USER_MOTOR_DEBUG_HALL_FOC           (1u)
#define USER_MOTOR_DEBUG_FORCE_ENABLE       (0u)
#define USER_MOTOR_OPEN_VOLTAGE_ALIGN       (8.0f)
#define USER_MOTOR_OPEN_VOLTAGE_RUN         (8.0f)
#define USER_MOTOR_OPEN_VOLTAGE_THETA_STEP  (0.0005f)
#define USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP   (0.0002f)

typedef enum
{
    E_USR_MOTOR_STARTUP_ALIGN = 0,
    E_USR_MOTOR_STARTUP_RUN = 1
} eUsrMotorStartupStateDef;

/** @brief 电机控制内部状态结构体（放置于 CCM SRAM 以保证快速访问） */
typedef struct tUsrMotorControlStateDef
{
    float f32OpenLoopTheta;     /**< Open-loop electrical angle, rad */
    float f32OpenLoopStep;      /**< Open-loop angle step, rad/control tick */
    float f32IqReference;       /**< Ramped q-axis current reference, A */
    float f32OpenVoltage;       /**< Ramped diagnostic open-loop voltage command */
    float f32FaultIa;           /**< Phase A current captured when over-current trips */
    float f32FaultIb;           /**< Phase B current captured when over-current trips */
    float f32FaultIc;           /**< Phase C current captured when over-current trips */
    uint32_t u32StartupCounter; /**< Startup alignment counter, control ticks */
    eUsrMotorStartupStateDef eStartupState; /**< Startup state: align first, then run */
    uint8_t u8OverCurrentFault; /**< Over-current fault flag, 1 = fault */
    uint8_t u8OverCurrentCount; /**< Over-current debounce counter */
    volatile uint8_t u8PowerOutputEnabled; /**< Power PWM outputs enabled after offset calibration */
} tUsrMotorControlStateDef;

static tUsrMotorControlStateDef tMotor USER_MOTOR_CCMRAM;

/** @brief 运行时诊断开关；volatile 防止编译器将零矢量分支折叠为不可达代码。 */
static volatile uint8_t u8DebugZeroVectorPwm = USER_MOTOR_DEBUG_ZERO_VECTOR_PWM;

/**
 * @brief  计算浮点数的绝对值
 * @param[in] f32Value  输入浮点数
 * @return 绝对值（非负）
 */
static float UsrMotorAbsFloat(float f32Value)
{
    return (f32Value >= 0.0f) ? f32Value : -f32Value;
}

/**
 * @brief  检查三相电流是否超过过流保护阈值
 * @param[in] f32Ia  A 相电流（A）
 * @param[in] f32Ib  B 相电流（A）
 * @param[in] f32Ic  C 相电流（A）
 * @retval 1  至少有一相电流超过限制值
 * @retval 0  所有相电流均在限制范围内
 * @note   过流阈值由 USER_MOTOR_PHASE_CURRENT_LIMIT 定义
 */
static uint8_t UsrMotorIsPhaseCurrentOverLimit(float f32Ia, float f32Ib, float f32Ic)
{
    return (UsrMotorAbsFloat(f32Ia) > USER_MOTOR_PHASE_CURRENT_LIMIT) ||
           (UsrMotorAbsFloat(f32Ib) > USER_MOTOR_PHASE_CURRENT_LIMIT) ||
           (UsrMotorAbsFloat(f32Ic) > USER_MOTOR_PHASE_CURRENT_LIMIT);
}

/**
 * @brief  Reset open-loop startup state.
 * @note   The next start locks rotor angle first, then begins open-loop run.
 */
static void UsrMotorResetStartup(void)
{
    tMotor.f32OpenLoopTheta = 0.0f;
    tMotor.f32OpenLoopStep = USER_MOTOR_THETA_STEP_INIT;
    tMotor.f32OpenVoltage = 0.0f;
    tMotor.u32StartupCounter = 0u;
    tMotor.eStartupState = E_USR_MOTOR_STARTUP_ALIGN;
    BspHallAngleInit();
}

static float UsrMotorRampFloat(float f32Current, float f32Target, float f32Step)
{
    if (f32Current < f32Target)
    {
        f32Current += f32Step;
        if (f32Current > f32Target)
        {
            f32Current = f32Target;
        }
    }
    else if (f32Current > f32Target)
    {
        f32Current -= f32Step;
        if (f32Current < f32Target)
        {
            f32Current = f32Target;
        }
    }

    return f32Current;
}

/**
 * @brief  Output a direct alpha-beta voltage vector for hardware diagnosis.
 * @note   This bypasses the current PI loop. Voltage unit is the same as BspPwmSetVoltageAbc percent scale.
 */
static void UsrMotorSetOpenLoopVoltageVector(float f32Theta, float f32Voltage)
{
    float SinTheta = sinf(f32Theta);
    float CosTheta = cosf(f32Theta);
    float Alpha = CosTheta * f32Voltage;
    float Beta = SinTheta * f32Voltage;
    float Ua = Alpha;
    float Ub = (-0.5f * Alpha) + (0.866025f * Beta);
    float Uc = (-0.5f * Alpha) - (0.866025f * Beta);

    BspPwmSetVoltageAbc(Ua, Ub, Uc);
}

#if (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u)
static void UsrMotorSetFixedVoltageVector(void)
{
    float Voltage = USER_MOTOR_DEBUG_FIXED_VOLTAGE;
    float NegativeHalf = -0.5f * Voltage;

#if (USER_MOTOR_DEBUG_FIXED_PHASE == 0u)
    BspPwmSetVoltageAbc(Voltage, NegativeHalf, NegativeHalf);
#elif (USER_MOTOR_DEBUG_FIXED_PHASE == 1u)
    BspPwmSetVoltageAbc(NegativeHalf, Voltage, NegativeHalf);
#else
    // BspPwmSetVoltageAbc(NegativeHalf, NegativeHalf, Voltage);
    BspPwmSetVoltageAbc(0, 0, 0);
#endif
}
#endif

#if (USER_MOTOR_DEBUG_OPEN_VOLTAGE == 0u)
/**
 * @brief  更新开环电角度（斜坡加速 + 角度累加）
 * @note   开环步长从 USER_MOTOR_THETA_STEP_INIT 开始，
 *         逐步递增至 USER_MOTOR_THETA_STEP_MAX，实现软启动加速。
 *         角度超过 2π 时绕回，保持 [0, 2π) 范围。
 *         在 10kHz 快速环中每次迭代调用一次。
 */
static void UsrMotorUpdateOpenLoopTheta(void)
{
    if (tMotor.f32OpenLoopStep < USER_MOTOR_THETA_STEP_MAX)
    {
        tMotor.f32OpenLoopStep += USER_MOTOR_THETA_STEP_INC;
    }

    tMotor.f32OpenLoopTheta += tMotor.f32OpenLoopStep;
    if (tMotor.f32OpenLoopTheta > USER_MOTOR_TWO_PI)
    {
        tMotor.f32OpenLoopTheta -= USER_MOTOR_TWO_PI;
    }
}
#endif

/**
 * @brief  斜坡式更新 q 轴电流参考值（软启动/软停止）
 * @param[in] f32Target  q 轴电流目标值（A）
 * @note   每次调用以 USER_MOTOR_IQ_REF_STEP 步长
 *         逐渐逼近目标值，防止电流突变。
 *         在 10kHz 快速环中每次迭代调用。
 */
static void UsrMotorUpdateIqReference(float f32Target)
{
    if (tMotor.f32IqReference < f32Target)
    {
        tMotor.f32IqReference += USER_MOTOR_IQ_REF_STEP;
        if (tMotor.f32IqReference > f32Target)
        {
            tMotor.f32IqReference = f32Target;
        }
    }
    else if (tMotor.f32IqReference > f32Target)
    {
        tMotor.f32IqReference -= USER_MOTOR_IQ_REF_STEP;
        if (tMotor.f32IqReference < f32Target)
        {
            tMotor.f32IqReference = f32Target;
        }
    }
}

/**
 * @brief  从电位器 ADC 值计算目标 q 轴电流
 * @return q 轴电流目标值（A），范围 0 ~ USER_MOTOR_IQ_REF_MAX
 * @note   电位器原始值低于死区（300 LSB）时返回 0。
 *         映射关系：Raw ∈ [300, 4095] → 0 ~ IQ_REF_MAX
 */
float UsrMotorGetIqRefTarget(void)
{
    uint16_t Raw = BspAdc2GetRaw(E_BSP_ADC2_POT);

    if (Raw <= USER_MOTOR_IQ_ADC_DEADZONE)
    {
        return 0.0f;
    }

    return ((float)(Raw - USER_MOTOR_IQ_ADC_DEADZONE) * USER_MOTOR_IQ_REF_MAX) /
           (USER_MOTOR_ADC_FULL_SCALE - (float)USER_MOTOR_IQ_ADC_DEADZONE);
}

/**
 * @brief  初始化电机控制状态
 * @note   清零电机控制状态结构体，设置开环角度步长初始值，
 *         并复位 FOC 运行时状态。
 *         在系统启动时由 main() 调用一次。
 */
void UsrMotorInit(void)
{
    memset(&tMotor, 0, sizeof(tMotor));
    UsrMotorResetStartup();
    UsrFocReset();
}

static void UsrMotorLogCalibrationFailure(HAL_StatusTypeDef Status)
{
    tBspAdcCalDebugDef Debug;
    uint8_t i;

    BspAdcGetCalDebug(&Debug);
    SEGGER_RTT_printf(0,
                      "MOTOR_START_FAIL,stage=CURRENT_OFFSET,status=%u,state=%u,retry=%u\r\n",
                      (unsigned int)Status,
                      (unsigned int)Debug.eState,
                      (unsigned int)Debug.u8RetryCount);

    for (i = 0u; i < BSP_ADC_INJECTED_CHANNELS; i++)
    {
        SEGGER_RTT_printf(0,
                          "CAL_CH%u,raw=%u,offset=%u,min=%u,max=%u,span=%u,drift=%d\r\n",
                          (unsigned int)i,
                          (unsigned int)BspAdcGetInjectedRaw(i),
                          (unsigned int)Debug.u16Offset[i],
                          (unsigned int)Debug.u16MinRaw[i],
                          (unsigned int)Debug.u16MaxRaw[i],
                          (unsigned int)Debug.u16Span[i],
                          (int)Debug.s16Drift[i]);
    }
}

static HAL_StatusTypeDef UsrMotorWaitForCurrentOffset(void)
{
    uint32_t StartTick = HAL_GetTick();

    while (1)
    {
        BspAdcProcess();

        if (BspAdcGetCalState() == E_BSP_ADC_CAL_READY)
        {
            SEGGER_RTT_WriteString(0, "Current offset calibration ready\r\n");
            return HAL_OK;
        }

        if (BspAdcGetCalState() == E_BSP_ADC_CAL_ERROR)
        {
            SEGGER_RTT_WriteString(0, "Current offset calibration failed\r\n");
            return HAL_ERROR;
        }

        if ((HAL_GetTick() - StartTick) >= USER_MOTOR_OFFSET_TIMEOUT_MS)
        {
            SEGGER_RTT_WriteString(0, "Current offset calibration timeout\r\n");
            return HAL_TIMEOUT;
        }
    }
}

/**
 * @brief  启动电机控制（ADC 偏置校准 + PWM 输出）
 * @retval HAL_OK      启动成功
 * @retval HAL_ERROR   校准失败
 * @retval HAL_TIMEOUT 校准超时
 * @retval HAL_BUSY    外设忙
 * @note   启动流程 (doc §18)：
 *         1. ADC1 自校准 + 启动注入组中断
 *         2. 启动 TIM1_CH4 内部触发
 *         3. 正常模式先以三相 50% 零矢量启动功率 PWM
 *         4. 在实际 PWM 开关环境中执行偏置校准
 *         5. 校准成功后才允许快速环更新 PWM
 * @see    HAL_ADCEx_InjectedConvCpltCallback → UsrMotorFastLoop
 */
HAL_StatusTypeDef UsrMotorStart(void)
{
    HAL_StatusTypeDef Status;

    tMotor.u8PowerOutputEnabled = 0u;

    /* 1. ADC1 自校准 + 启动注入组中断 */
    Status = BspAdcStartInjected();
    if (Status != HAL_OK)
    {
        SEGGER_RTT_printf(0,
                          "MOTOR_START_FAIL,stage=ADC_INJECTED,status=%u\r\n",
                          (unsigned int)Status);
        return Status;
    }

    /* 2. 临时停止注入采样，使用 ADC1 普通轮询预采样零电流偏置。 */
    // BspAdcPreOffset();

    /* 3. 启动 TIM1_CH4 内部触发。 */
    Status = BspPwmStartAdcTrigger();
    if (Status != HAL_OK)
    {
        SEGGER_RTT_printf(0,
                          "MOTOR_START_FAIL,stage=ADC_TRIGGER,status=%u\r\n",
                          (unsigned int)Status);
        return Status;
    }

#if (USER_MOTOR_DEBUG_ADC_ONLY == 0u)
    /*
     * 4. 在真实开关环境下校准：
     *    先固定三相 50% 零矢量并启动功率 PWM。power output enabled
     *    仍保持为 0，校准完成前快速环不能输出非零电压。
     */
    BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
    Status = BspPwmStartPowerOutputs();
    if (Status != HAL_OK)
    {
        SEGGER_RTT_printf(0,
                          "MOTOR_START_FAIL,stage=POWER_PWM,status=%u\r\n",
                          (unsigned int)Status);
        BspPwmStop();
        return Status;
    }
#endif

    /* 5. 等待模拟链路稳定，并在当前采样环境中累计零电流偏置。 */
    BspAdcCalibrationStart();

    /* 6. 等待校准完成。 */
    Status = UsrMotorWaitForCurrentOffset();
    if (Status != HAL_OK)
    {
        BspPwmStop();
        UsrMotorLogCalibrationFailure(Status);
        return Status;
    }

#if (USER_MOTOR_DEBUG_ADC_ONLY != 0u)
    /*
     * ADC-only diagnostic mode:
     * - TIM1 CH4 remains running and continues to trigger ADC1 injected conversions.
     * - TIM1 CH1/CH2/CH3 and complementary outputs are never started.
     * - Keep the power-output-enabled flag cleared so the FOC fast loop cannot drive PWM.
     */
    SEGGER_RTT_WriteString(0, "Motor startup ready (ADC only)\r\n");
    return HAL_OK;
#else
    /* 功率 PWM 已在校准前启动；校准成功后才放行快速环。 */
    tMotor.u8PowerOutputEnabled = 1u;
#if (USER_MOTOR_DEBUG_ZERO_VECTOR_PWM != 0u)
    SEGGER_RTT_WriteString(0, "Motor startup ready (zero-vector PWM)\r\n");
#else
    SEGGER_RTT_WriteString(0, "Motor startup ready\r\n");
#endif
    return HAL_OK;
#endif
}

/**
 * @brief  电机快速控制环（20kHz，在 ADC 中断中执行）
 * @note   放置在 .fastcode 段（CCM SRAM），以保证零等待执行。
 *         执行流程：
 *         1. 等待 ADC 零电流偏移校准完成
 *         2. 从电位器获取目标 Iq 参考值，斜坡更新当前 Iq
 *         3. Iq ≈ 0 时输出零电压（电机静止）
 *         4. 过流故障时输出零电压
 *         5. 采样三相电流，检查过流
 *         6. 更新开环角度
 *         7. 执行 FOC 电流环
 *         8. 更新三相 PWM 占空比
 * @warning 此函数在中断上下文中执行，必须保持高效，
 *          禁止阻塞操作、RTT 打印或慢速外设轮询。
 */
USER_MOTOR_FAST_CODE void UsrMotorFastLoop(void)
{
    tFocCurrentLoopInputDef Input;
    tFocCurrentLoopOutputDef Output;
    float IqReferenceTarget;

#if (USER_MOTOR_DEBUG_OPEN_VOLTAGE != 0u) || (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u) || (USER_MOTOR_DEBUG_HALL_FOC != 0u)
    (void)Output;
#endif

    if ((BspAdcIsCurrentOffsetReady() == 0u) ||
        (tMotor.u8PowerOutputEnabled == 0u))
    {
        BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
        return;
    }

    if (u8DebugZeroVectorPwm != 0u)
    {
        /* 诊断模式：保持三相相同占空比，仅观察运行态零偏和采样噪声。 */
        BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
        return;
    }

#if (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u) && (USER_MOTOR_DEBUG_FORCE_ENABLE != 0u)
    IqReferenceTarget = USER_MOTOR_IQ_REF_MAX;
#else
    IqReferenceTarget = UsrMotorGetIqRefTarget();
#endif
    UsrMotorUpdateIqReference(IqReferenceTarget);

    if (tMotor.f32IqReference < USER_MOTOR_IQ_STOP_THRESHOLD)
    {
        tMotor.u8OverCurrentFault = 0u;
        UsrMotorResetStartup();
        UsrFocReset();
        BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
        return;
    }

    if (tMotor.u8OverCurrentFault != 0u)
    {
        UsrMotorResetStartup();
        UsrFocReset();
        BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
        return;
    }

    /* 采样窗口无效时保持上一周期 PWM 输出，跳过本周期电流环 (doc §6)。
     * 必须在 OVC 检查之前执行：采样窗口无效时电流值为开关瞬态噪声，
     * 不能用于过流判断，否则会误触发故障。 */
    if (BspAdcIsSampleValid() == 0u)
    {
        return;
    }

    Input.f32Ia = BspAdcGetIa();
    Input.f32Ib = BspAdcGetIb();
    Input.f32Ic = BspAdcGetIc();
    if (UsrMotorIsPhaseCurrentOverLimit(Input.f32Ia, Input.f32Ib, Input.f32Ic) != 0u)
    {
        /* 过流去抖：连续 USER_MOTOR_OVC_DEBOUNCE_COUNT 次过流才触发故障，
         * 避免单次采样噪声导致误触发。 */
        tMotor.u8OverCurrentCount++;
        if (tMotor.u8OverCurrentCount >= USER_MOTOR_OVC_DEBOUNCE_COUNT)
        {
            tMotor.f32FaultIa = Input.f32Ia;
            tMotor.f32FaultIb = Input.f32Ib;
            tMotor.f32FaultIc = Input.f32Ic;
            tMotor.u8OverCurrentFault = 1u;
            UsrMotorResetStartup();
            UsrFocReset();
            BspPwmSetVoltageAbc(0.0f, 0.0f, 0.0f);
        }
        return;
    }
    tMotor.u8OverCurrentCount = 0u;

    /* Hall 角度跟踪更新（所有模式都调用）*/
    BspHallAngleUpdate();

#if (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u)
    UsrMotorSetFixedVoltageVector();
    return;
#endif

#if (USER_MOTOR_DEBUG_HALL_FOC != 0u)
    if (tMotor.eStartupState == E_USR_MOTOR_STARTUP_ALIGN)
    {
        /* ALIGN: 开环电压对齐转子，同时 Hall 角度跟踪开始累计 */
        tMotor.f32OpenVoltage = UsrMotorRampFloat(tMotor.f32OpenVoltage,
                                                   USER_MOTOR_OPEN_VOLTAGE_ALIGN,
                                                   USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP);
        UsrMotorSetOpenLoopVoltageVector(0.0f, tMotor.f32OpenVoltage);
        tMotor.u32StartupCounter++;
        if (tMotor.u32StartupCounter >= USER_MOTOR_ALIGN_TICKS)
        {
            tMotor.eStartupState = E_USR_MOTOR_STARTUP_RUN;
            tMotor.f32OpenLoopTheta = 0.0f;
            tMotor.f32OpenLoopStep = USER_MOTOR_OPEN_VOLTAGE_THETA_STEP;
            tMotor.f32OpenVoltage = 0.0f;
            UsrFocReset();
        }
        return;
    }

    /* RUN 阶段：Hall 角度有效后切入 FOC 电流闭环 */
    if (BspHallIsAngleValid() == 0u)
    {
        /* Hall 尚未有效（跳变次数 < 2）：继续开环电压拖动 */
        tMotor.f32OpenLoopTheta += USER_MOTOR_OPEN_VOLTAGE_THETA_STEP;
        if (tMotor.f32OpenLoopTheta > USER_MOTOR_TWO_PI)
        {
            tMotor.f32OpenLoopTheta -= USER_MOTOR_TWO_PI;
        }
        tMotor.f32OpenVoltage = UsrMotorRampFloat(tMotor.f32OpenVoltage,
                                                   USER_MOTOR_OPEN_VOLTAGE_RUN,
                                                   USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP);
        UsrMotorSetOpenLoopVoltageVector(tMotor.f32OpenLoopTheta, tMotor.f32OpenVoltage);
        return;
    }

    /* Hall 有效：用 Hall 电角度做 FOC 电流闭环 */
    Input.f32Theta = BspHallGetElectricalAngle();
    Input.f32IdReference = 0.0f;
    Input.f32IqReference = tMotor.f32IqReference;
    UsrFocRunCurrentLoop(&Input, &Output);
    BspPwmSetVoltageAbc(Output.tVoltage.f32Ua, Output.tVoltage.f32Ub, Output.tVoltage.f32Uc);
    return;
#elif (USER_MOTOR_DEBUG_OPEN_VOLTAGE != 0u)
    if (tMotor.eStartupState == E_USR_MOTOR_STARTUP_ALIGN)
    {
        tMotor.f32OpenVoltage = UsrMotorRampFloat(tMotor.f32OpenVoltage,
                                                   USER_MOTOR_OPEN_VOLTAGE_ALIGN,
                                                   USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP);
        UsrMotorSetOpenLoopVoltageVector(0.0f, tMotor.f32OpenVoltage);

        tMotor.u32StartupCounter++;
        if (tMotor.u32StartupCounter >= USER_MOTOR_ALIGN_TICKS)
        {
            tMotor.eStartupState = E_USR_MOTOR_STARTUP_RUN;
            tMotor.f32OpenLoopTheta = 0.0f;
            tMotor.f32OpenLoopStep = USER_MOTOR_OPEN_VOLTAGE_THETA_STEP;
            tMotor.f32OpenVoltage = 0.0f;
            UsrFocReset();
        }
        return;
    }

    tMotor.f32OpenLoopTheta += USER_MOTOR_OPEN_VOLTAGE_THETA_STEP;
    if (tMotor.f32OpenLoopTheta > USER_MOTOR_TWO_PI)
    {
        tMotor.f32OpenLoopTheta -= USER_MOTOR_TWO_PI;
    }
    tMotor.f32OpenVoltage = UsrMotorRampFloat(tMotor.f32OpenVoltage,
                                               USER_MOTOR_OPEN_VOLTAGE_RUN,
                                               USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP);
    UsrMotorSetOpenLoopVoltageVector(tMotor.f32OpenLoopTheta, tMotor.f32OpenVoltage);
    return;
#else
    if (tMotor.eStartupState == E_USR_MOTOR_STARTUP_ALIGN)
    {
        Input.f32Theta = 0.0f;
        Input.f32IdReference = USER_MOTOR_ALIGN_ID_REF;
        Input.f32IqReference = 0.0f;

        UsrFocRunCurrentLoop(&Input, &Output);
        BspPwmSetVoltageAbc(Output.tVoltage.f32Ua, Output.tVoltage.f32Ub, Output.tVoltage.f32Uc);

        tMotor.u32StartupCounter++;
        if (tMotor.u32StartupCounter >= USER_MOTOR_ALIGN_TICKS)
        {
            tMotor.eStartupState = E_USR_MOTOR_STARTUP_RUN;
            tMotor.f32OpenLoopTheta = 0.0f;
            tMotor.f32OpenLoopStep = USER_MOTOR_THETA_STEP_INIT;
            UsrFocReset();
        }
        return;
    }

    UsrMotorUpdateOpenLoopTheta();

    Input.f32Theta = tMotor.f32OpenLoopTheta;
    Input.f32IdReference = 0.0f;
    Input.f32IqReference = tMotor.f32IqReference;

    UsrFocRunCurrentLoop(&Input, &Output);
    BspPwmSetVoltageAbc(Output.tVoltage.f32Ua, Output.tVoltage.f32Ub, Output.tVoltage.f32Uc);
#endif
}

/**
 * @brief  获取当前开环电角度
 * @return 电角度（rad），范围 [0, 2π)
 * @note   用于 FOC 坐标变换和调试显示
 */
float UsrMotorGetOpenLoopTheta(void)
{
    return tMotor.f32OpenLoopTheta;
}

/**
 * @brief  获取当前 q 轴电流参考值（斜坡输出值）
 * @return 当前 Iq 参考值（A）
 * @note   此值为斜坡逼近后的实际输出值，可能不等于目标值
 * @see    UsrMotorGetIqRefTarget
 */
float UsrMotorGetIqRef(void)
{
    return tMotor.f32IqReference;
}

/**
 * @brief  查询是否发生过流故障
 * @retval 1  发生过流，电机已停止
 * @retval 0  无过流故障
 * @note   过流故障在 Iq 参考值降至阈值以下后自动清除
 */
uint8_t UsrMotorIsOverCurrentFault(void)
{
    return tMotor.u8OverCurrentFault;
}

float UsrMotorGetFaultIa(void)
{
    return tMotor.f32FaultIa;
}

float UsrMotorGetFaultIb(void)
{
    return tMotor.f32FaultIb;
}

float UsrMotorGetFaultIc(void)
{
    return tMotor.f32FaultIc;
}
