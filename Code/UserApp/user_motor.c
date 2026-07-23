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
#include "user_foc.h"
#include <math.h>

#define USER_MOTOR_PI              3.14159265f
#define USER_MOTOR_TWO_PI          (2.0f * USER_MOTOR_PI)
#define USER_MOTOR_IQ_REF_MAX      0.5f
#define USER_MOTOR_IQ_REF_STEP     0.002f
#define USER_MOTOR_FAST_LOOP_HZ     20000u
#define USER_MOTOR_ADC_FULL_SCALE  4095.0f
#define USER_MOTOR_IQ_ADC_DEADZONE 300u
#define USER_MOTOR_THETA_STEP_INIT 0.001f
#define USER_MOTOR_THETA_STEP_MAX  0.006f
#define USER_MOTOR_THETA_STEP_INC  0.0000004f
#define USER_MOTOR_IQ_STOP_THRESHOLD 0.02f
#define USER_MOTOR_PHASE_CURRENT_LIMIT 3.0f
#define USER_MOTOR_ALIGN_TIME_MS    800u
#define USER_MOTOR_ALIGN_TICKS      ((USER_MOTOR_FAST_LOOP_HZ * USER_MOTOR_ALIGN_TIME_MS) / 1000u)
#define USER_MOTOR_ALIGN_ID_REF     0.4f
#define USER_MOTOR_OFFSET_TIMEOUT_MS 500u

/* Temporary diagnostic modes. Keep only one active while checking phase wiring. */
#define USER_MOTOR_DEBUG_ADC_ONLY           0u      /* 1: keep CH1/CH2/CH3 and CH1N/CH2N/CH3N disabled; CH4 still triggers ADC. */
#define USER_MOTOR_DEBUG_ZERO_VECTOR_PWM     1u      /* 1: enable power PWM at 50%/50%/50% for sampling-point verification. */
#define USER_MOTOR_DEBUG_FIXED_VECTOR       0u
#define USER_MOTOR_DEBUG_FIXED_PHASE        2u      /* 0=A+, 1=B+, 2=C+ */
#define USER_MOTOR_DEBUG_FIXED_VOLTAGE      1.5f    /* Same percent-scale unit as BspPwm_SetVoltageABC. */
#define USER_MOTOR_DEBUG_OPEN_VOLTAGE       0u
#define USER_MOTOR_DEBUG_FORCE_ENABLE       0u
#define USER_MOTOR_OPEN_VOLTAGE_ALIGN       2.0f
#define USER_MOTOR_OPEN_VOLTAGE_RUN         2.0f
#define USER_MOTOR_OPEN_VOLTAGE_THETA_STEP  0.002f
#define USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP   0.0002f

typedef enum
{
    USER_MOTOR_STARTUP_ALIGN = 0,
    USER_MOTOR_STARTUP_RUN = 1
} UserMotor_StartupState_t;

/** @brief 电机控制内部状态结构体（放置于 CCM SRAM 以保证快速访问） */
typedef struct
{
    float open_loop_theta;     /**< Open-loop electrical angle, rad */
    float open_loop_step;      /**< Open-loop angle step, rad/control tick */
    float iq_ref;              /**< Ramped q-axis current reference, A */
    float open_voltage;         /**< Ramped diagnostic open-loop voltage command */
    float fault_ia;             /**< Phase A current captured when over-current trips */
    float fault_ib;             /**< Phase B current captured when over-current trips */
    float fault_ic;             /**< Phase C current captured when over-current trips */
    uint32_t startup_counter;   /**< Startup alignment counter, control ticks */
    UserMotor_StartupState_t startup_state; /**< Startup state: align first, then run */
    uint8_t over_current_fault; /**< Over-current fault flag, 1 = fault */
    volatile uint8_t power_output_enabled; /**< Power PWM outputs enabled after offset calibration */
} UserMotor_ControlState_t;

static UserMotor_ControlState_t s_motor USER_MOTOR_CCMRAM;

/**
 * @brief  计算浮点数的绝对值
 * @param[in] value  输入浮点数
 * @return 绝对值（非负）
 */
static float UserMotor_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief  检查三相电流是否超过过流保护阈值
 * @param[in] ia  A 相电流（A）
 * @param[in] ib  B 相电流（A）
 * @param[in] ic  C 相电流（A）
 * @retval 1  至少有一相电流超过限制值
 * @retval 0  所有相电流均在限制范围内
 * @note   过流阈值由 USER_MOTOR_PHASE_CURRENT_LIMIT 定义
 */
static uint8_t UserMotor_IsPhaseCurrentOverLimit(float ia, float ib, float ic)
{
    return (UserMotor_AbsFloat(ia) > USER_MOTOR_PHASE_CURRENT_LIMIT) ||
           (UserMotor_AbsFloat(ib) > USER_MOTOR_PHASE_CURRENT_LIMIT) ||
           (UserMotor_AbsFloat(ic) > USER_MOTOR_PHASE_CURRENT_LIMIT);
}

/**
 * @brief  Reset open-loop startup state.
 * @note   The next start locks rotor angle first, then begins open-loop run.
 */
static void UserMotor_ResetStartup(void)
{
    s_motor.open_loop_theta = 0.0f;
    s_motor.open_loop_step = USER_MOTOR_THETA_STEP_INIT;
    s_motor.open_voltage = 0.0f;
    s_motor.startup_counter = 0u;
    s_motor.startup_state = USER_MOTOR_STARTUP_ALIGN;
}

static float UserMotor_RampFloat(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

/**
 * @brief  Output a direct alpha-beta voltage vector for hardware diagnosis.
 * @note   This bypasses the current PI loop. Voltage unit is the same as BspPwm_SetVoltageABC percent scale.
 */
static void UserMotor_SetOpenLoopVoltageVector(float theta, float voltage)
{
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float alpha = cos_theta * voltage;
    float beta = sin_theta * voltage;
    float ua = alpha;
    float ub = (-0.5f * alpha) + (0.866025f * beta);
    float uc = (-0.5f * alpha) - (0.866025f * beta);

    BspPwm_SetVoltageABC(ua, ub, uc);
}

#if (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u)
static void UserMotor_SetFixedVoltageVector(void)
{
    float voltage = USER_MOTOR_DEBUG_FIXED_VOLTAGE;
    float negative_half = -0.5f * voltage;

#if (USER_MOTOR_DEBUG_FIXED_PHASE == 0u)
    BspPwm_SetVoltageABC(voltage, negative_half, negative_half);
#elif (USER_MOTOR_DEBUG_FIXED_PHASE == 1u)
    BspPwm_SetVoltageABC(negative_half, voltage, negative_half);
#else
    // BspPwm_SetVoltageABC(negative_half, negative_half, voltage);
    BspPwm_SetVoltageABC(0, 0, 0);
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
static void UserMotor_UpdateOpenLoopTheta(void)
{
    if (s_motor.open_loop_step < USER_MOTOR_THETA_STEP_MAX)
    {
        s_motor.open_loop_step += USER_MOTOR_THETA_STEP_INC;
    }

    s_motor.open_loop_theta += s_motor.open_loop_step;
    if (s_motor.open_loop_theta > USER_MOTOR_TWO_PI)
    {
        s_motor.open_loop_theta -= USER_MOTOR_TWO_PI;
    }
}
#endif

/**
 * @brief  斜坡式更新 q 轴电流参考值（软启动/软停止）
 * @param[in] target  q 轴电流目标值（A）
 * @note   每次调用以 USER_MOTOR_IQ_REF_STEP 步长
 *         逐渐逼近目标值，防止电流突变。
 *         在 10kHz 快速环中每次迭代调用。
 */
static void UserMotor_UpdateIqReference(float target)
{
    if (s_motor.iq_ref < target)
    {
        s_motor.iq_ref += USER_MOTOR_IQ_REF_STEP;
        if (s_motor.iq_ref > target)
        {
            s_motor.iq_ref = target;
        }
    }
    else if (s_motor.iq_ref > target)
    {
        s_motor.iq_ref -= USER_MOTOR_IQ_REF_STEP;
        if (s_motor.iq_ref < target)
        {
            s_motor.iq_ref = target;
        }
    }
}

/**
 * @brief  从电位器 ADC 值计算目标 q 轴电流
 * @return q 轴电流目标值（A），范围 0 ~ USER_MOTOR_IQ_REF_MAX
 * @note   电位器原始值低于死区（300 LSB）时返回 0。
 *         映射关系：raw ∈ [300, 4095] → 0 ~ IQ_REF_MAX
 */
float UserMotor_GetIqRefTarget(void)
{
    uint16_t raw = BspAdc2_GetRaw(BSP_ADC2_POT);

    if (raw <= USER_MOTOR_IQ_ADC_DEADZONE)
    {
        return 0.0f;
    }

    return ((float)(raw - USER_MOTOR_IQ_ADC_DEADZONE) * USER_MOTOR_IQ_REF_MAX) /
           (USER_MOTOR_ADC_FULL_SCALE - (float)USER_MOTOR_IQ_ADC_DEADZONE);
}

/**
 * @brief  初始化电机控制状态
 * @note   清零电机控制状态结构体，设置开环角度步长初始值，
 *         并复位 FOC 运行时状态。
 *         在系统启动时由 main() 调用一次。
 */
void UserMotor_Init(void)
{
    memset(&s_motor, 0, sizeof(s_motor));
    UserMotor_ResetStartup();
    FOC_Reset();
}

static HAL_StatusTypeDef UserMotor_WaitForCurrentOffset(void)
{
    uint32_t start_tick = HAL_GetTick();

    while (1)
    {
        BspAdc_Process();

        if (BspAdc_GetCalState() == CS_CAL_READY)
        {
            return HAL_OK;
        }

        if (BspAdc_GetCalState() == CS_CAL_ERROR)
        {
            return HAL_ERROR;
        }

        if ((HAL_GetTick() - start_tick) >= USER_MOTOR_OFFSET_TIMEOUT_MS)
        {
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
 *         2. 启动 TIM1_CH4 内部触发（功率 PWM 关闭）
 *         3. 启动偏置校准状态机（丢弃 64 点，累计 512 点）
 *         4. 等待校准完成（主循环驱动 BspAdc_Process）
 *         5. 校准成功后启动功率 PWM 输出
 * @see    HAL_ADCEx_InjectedConvCpltCallback → UserMotor_FastLoop
 */
HAL_StatusTypeDef UserMotor_Start(void)
{
    HAL_StatusTypeDef status;

    s_motor.power_output_enabled = 0u;

    /* 1. ADC1 自校准 + 启动注入组中断 */
    status = BspAdc_StartInjected();
    if (status != HAL_OK)
    {
        return status;
    }

    /* 2. 启动 TIM1_CH4 内部触发（功率 PWM 关闭） */
    status = BspPwm_StartAdcTrigger();
    if (status != HAL_OK)
    {
        return status;
    }

    /* 3. 启动偏置校准状态机（桥关闭，仅 TIM1 计数 + CH4 触发 + ADC 采样） */
    BspAdc_CalibrationStart();

    /* 4. 等待校准完成 */
    status = UserMotor_WaitForCurrentOffset();
    if (status != HAL_OK)
    {
        BspPwm_Stop();
        return status;
    }

#if (USER_MOTOR_DEBUG_ADC_ONLY != 0u)
    /*
     * ADC-only diagnostic mode:
     * - TIM1 CH4 remains running and continues to trigger ADC1 injected conversions.
     * - TIM1 CH1/CH2/CH3 and complementary outputs are never started.
     * - Keep power_output_enabled cleared so the FOC fast loop cannot drive PWM.
     */
    return HAL_OK;
#else
    /* 5. 校准成功，以零电压启动功率输出 */
    BspPwm_SetVoltageABC(0.0f, 0.0f, 0.0f);
    status = BspPwm_StartPowerOutputs();
    if (status != HAL_OK)
    {
        BspPwm_Stop();
        return status;
    }

    s_motor.power_output_enabled = 1u;
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
USER_MOTOR_FAST_CODE void UserMotor_FastLoop(void)
{
    FOC_CurrentLoopInput_t input;
    FOC_CurrentLoopOutput_t output;
    float iq_ref_target;

#if (USER_MOTOR_DEBUG_OPEN_VOLTAGE != 0u) || (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u)
    (void)output;
#endif

    if ((BspAdc_IsCurrentOffsetReady() == 0u) ||
        (s_motor.power_output_enabled == 0u))
    {
        BspPwm_SetVoltageABC(0.0f, 0.0f, 0.0f);
        return;
    }

#if (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u) && (USER_MOTOR_DEBUG_FORCE_ENABLE != 0u)
    iq_ref_target = USER_MOTOR_IQ_REF_MAX;
#else
    iq_ref_target = UserMotor_GetIqRefTarget();
#endif
    UserMotor_UpdateIqReference(iq_ref_target);

    if (s_motor.iq_ref < USER_MOTOR_IQ_STOP_THRESHOLD)
    {
        s_motor.over_current_fault = 0u;
        UserMotor_ResetStartup();
        FOC_Reset();
        BspPwm_SetVoltageABC(0.0f, 0.0f, 0.0f);
        return;
    }

    if (s_motor.over_current_fault != 0u)
    {
        UserMotor_ResetStartup();
        FOC_Reset();
        BspPwm_SetVoltageABC(0.0f, 0.0f, 0.0f);
        return;
    }

    input.ia = BspAdc_GetIa();
    input.ib = BspAdc_GetIb();
    input.ic = BspAdc_GetIc();
    if (UserMotor_IsPhaseCurrentOverLimit(input.ia, input.ib, input.ic) != 0u)
    {
        s_motor.fault_ia = input.ia;
        s_motor.fault_ib = input.ib;
        s_motor.fault_ic = input.ic;
        s_motor.over_current_fault = 1u;
        UserMotor_ResetStartup();
        FOC_Reset();
        BspPwm_SetVoltageABC(0.0f, 0.0f, 0.0f);
        return;
    }

    /* 采样窗口无效时保持上一周期 PWM 输出，跳过本周期电流环 (doc §6) */
    if (BspAdc_IsSampleValid() == 0u)
    {
        return;
    }

#if (USER_MOTOR_DEBUG_FIXED_VECTOR != 0u)
    UserMotor_SetFixedVoltageVector();
    return;
#endif

#if (USER_MOTOR_DEBUG_OPEN_VOLTAGE != 0u)
    if (s_motor.startup_state == USER_MOTOR_STARTUP_ALIGN)
    {
        s_motor.open_voltage = UserMotor_RampFloat(s_motor.open_voltage,
                                                   USER_MOTOR_OPEN_VOLTAGE_ALIGN,
                                                   USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP);
        UserMotor_SetOpenLoopVoltageVector(0.0f, s_motor.open_voltage);

        s_motor.startup_counter++;
        if (s_motor.startup_counter >= USER_MOTOR_ALIGN_TICKS)
        {
            s_motor.startup_state = USER_MOTOR_STARTUP_RUN;
            s_motor.open_loop_theta = 0.0f;
            s_motor.open_loop_step = USER_MOTOR_OPEN_VOLTAGE_THETA_STEP;
            s_motor.open_voltage = 0.0f;
            FOC_Reset();
        }
        return;
    }

    s_motor.open_loop_theta += USER_MOTOR_OPEN_VOLTAGE_THETA_STEP;
    if (s_motor.open_loop_theta > USER_MOTOR_TWO_PI)
    {
        s_motor.open_loop_theta -= USER_MOTOR_TWO_PI;
    }
    s_motor.open_voltage = UserMotor_RampFloat(s_motor.open_voltage,
                                               USER_MOTOR_OPEN_VOLTAGE_RUN,
                                               USER_MOTOR_OPEN_VOLTAGE_RAMP_STEP);
    UserMotor_SetOpenLoopVoltageVector(s_motor.open_loop_theta, s_motor.open_voltage);
    return;
#else
    if (s_motor.startup_state == USER_MOTOR_STARTUP_ALIGN)
    {
        input.theta = 0.0f;
        input.id_ref = USER_MOTOR_ALIGN_ID_REF;
        input.iq_ref = 0.0f;

        FOC_RunCurrentLoop(&input, &output);
        BspPwm_SetVoltageABC(output.voltage.ua, output.voltage.ub, output.voltage.uc);

        s_motor.startup_counter++;
        if (s_motor.startup_counter >= USER_MOTOR_ALIGN_TICKS)
        {
            s_motor.startup_state = USER_MOTOR_STARTUP_RUN;
            s_motor.open_loop_theta = 0.0f;
            s_motor.open_loop_step = USER_MOTOR_THETA_STEP_INIT;
            FOC_Reset();
        }
        return;
    }

    UserMotor_UpdateOpenLoopTheta();

    input.theta = s_motor.open_loop_theta;
    input.id_ref = 0.0f;
    input.iq_ref = s_motor.iq_ref;

    FOC_RunCurrentLoop(&input, &output);
    BspPwm_SetVoltageABC(output.voltage.ua, output.voltage.ub, output.voltage.uc);
#endif
}

/**
 * @brief  获取当前开环电角度
 * @return 电角度（rad），范围 [0, 2π)
 * @note   用于 FOC 坐标变换和调试显示
 */
float UserMotor_GetOpenLoopTheta(void)
{
    return s_motor.open_loop_theta;
}

/**
 * @brief  获取当前 q 轴电流参考值（斜坡输出值）
 * @return 当前 Iq 参考值（A）
 * @note   此值为斜坡逼近后的实际输出值，可能不等于目标值
 * @see    UserMotor_GetIqRefTarget
 */
float UserMotor_GetIqRef(void)
{
    return s_motor.iq_ref;
}

/**
 * @brief  查询是否发生过流故障
 * @retval 1  发生过流，电机已停止
 * @retval 0  无过流故障
 * @note   过流故障在 Iq 参考值降至阈值以下后自动清除
 */
uint8_t UserMotor_IsOverCurrentFault(void)
{
    return s_motor.over_current_fault;
}

float UserMotor_GetFaultIa(void)
{
    return s_motor.fault_ia;
}

float UserMotor_GetFaultIb(void)
{
    return s_motor.fault_ib;
}

float UserMotor_GetFaultIc(void)
{
    return s_motor.fault_ic;
}
