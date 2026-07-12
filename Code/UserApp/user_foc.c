/**
 * @file    user_foc.c
 * @brief   FOC 电流闭环控制实现
 *******************************************************************************
 * @note    实现磁场定向控制（FOC）的核心数学运算：
 *          - Clarke/Park 正变换与逆变换
 *          - dq 轴 PI 控制器（带积分抗饱和）
 *          - dq 电压限幅
 *
 *          控制周期由 ADC 注入转换触发，标称频率 20kHz（50us）。
 *          三相电流采样后经变换至 dq 坐标系，
 *          PI 控制器输出 dq 电压，再经逆变换得到三相调制电压。
 *******************************************************************************
 */

#include "user_foc.h"
#include <math.h>

/** @brief FOC 控制周期（秒），对应 20kHz */
#define FOC_CONTROL_TS              0.00005f

/** @brief dq 电压调制限幅值 */
#define FOC_MODULATION_LIMIT        20.0f

/** @brief q 轴电压符号（1.0 = 正方向） */
#define FOC_Q_VOLTAGE_SIGN          (1.0f)

/** @brief 静止坐标系电流（αβ 轴） */
typedef struct
{
    float alpha;  /**< α 轴电流分量 */
    float beta;   /**< β 轴电流分量 */
} FOC_AlphaBeta_t;

/** @brief 同步旋转坐标系电压/电流（dq 轴） */
typedef struct
{
    float d;  /**< d 轴分量（励磁） */
    float q;  /**< q 轴分量（转矩） */
} FOC_Dq_t;

/** @brief PI 控制器配置参数 */
typedef struct
{
    float kp;           /**< 比例增益 */
    float ki;           /**< 积分增益 */
    float max_output;   /**< 输出限幅值 */
} FOC_PiConfig_t;

/** @brief PI 控制器运行时状态 */
typedef struct
{
    float integral;  /**< 积分累加值 */
    float output;    /**< 当前输出值 */
} FOC_PiState_t;

/** @brief FOC 全局配置参数 */
typedef struct
{
    float sample_period;        /**< 控制周期（秒） */
    float modulation_limit;     /**< 调制电压限幅值 */
    float q_voltage_sign;       /**< q 轴电压方向符号 */
    FOC_PiConfig_t pi_d;        /**< d 轴 PI 控制器参数 */
    FOC_PiConfig_t pi_q;        /**< q 轴 PI 控制器参数 */
} FOC_Config_t;

/** @brief dq 电流参考值 */
typedef struct
{
    float d;  /**< d 轴电流参考值（A），通常 Id_ref = 0 */
    float q;  /**< q 轴电流参考值（A），决定转矩输出 */
} FOC_Reference_t;

/** @brief FOC 单次运算运行时数据（所有中间变量） */
typedef struct
{
    float ia;                /**< A 相采样电流（A） */
    float ib;                /**< B 相采样电流（A） */
    float ic;                /**< C 相采样电流（A） */
    float theta;             /**< 电角度（rad） */
    float sin_theta;         /**< 电角度的正弦值 */
    float cos_theta;         /**< 电角度的余弦值 */
    FOC_AlphaBeta_t current_ab; /**< αβ 轴电流（Clarke 变换后） */
    FOC_Dq_t current_dq;       /**< dq 轴电流（Park 变换后） */
    FOC_PiState_t pi_d;        /**< d 轴 PI 控制器运行时状态 */
    FOC_PiState_t pi_q;        /**< q 轴 PI 控制器运行时状态 */
    FOC_Dq_t voltage_dq;       /**< dq 轴电压（PI 输出） */
    FOC_AlphaBeta_t voltage_ab; /**< αβ 轴电压（逆 Park 变换后） */
    ThreePhaseVoltage_t voltage_abc; /**< 三相调制电压（逆 Clarke 变换后） */
} FOC_Runtime_t;

/** @brief FOC 全局上下文（配置 + 参考值 + 运行时） */
typedef struct
{
    FOC_Config_t config;         /**< 控制参数（不可在快速环中修改） */
    FOC_Reference_t reference;   /**< dq 电流参考值 */
    FOC_Runtime_t runtime;       /**< 运行时数据，每次迭代更新 */
} FOC_Context_t;

static FOC_Context_t s_foc = {
    .config = {
        .sample_period = FOC_CONTROL_TS,
        .modulation_limit = FOC_MODULATION_LIMIT,
        .q_voltage_sign = FOC_Q_VOLTAGE_SIGN,
        .pi_d = {
            .kp = 0.35f,
            .ki = 2.0f,
            .max_output = FOC_MODULATION_LIMIT
        },
        .pi_q = {
            .kp = 0.5f,
            .ki = 1.5f,
            .max_output = FOC_MODULATION_LIMIT
        }
    },
    .reference = {0.0f, 0.0f}
};

/**
 * @brief  Clarke 变换：三相电流 (abc) → 静止两相 (αβ)
 * @param[in] ia  A 相电流（A）
 * @param[in] ib  B 相电流（A）
 * @param[in] ic  C 相电流（A）
 * @return αβ 轴电流分量
 * @note   采用等幅值变换，β 系数为 1/√3 ≈ 0.57735
 */
static FOC_AlphaBeta_t FOC_ClarkeTransform(float ia, float ib, float ic)
{
    FOC_AlphaBeta_t result;
    float zero_sequence = (ia + ib + ic) * 0.333333f;

    /* Three motor phase currents should sum to zero.  Remove common-mode
     * sampling error before Clarke transform; fixed-vector tests show a
     * noticeable Ia+Ib+Ic offset caused by the low-side sampling window.
     */
    ia -= zero_sequence;
    ib -= zero_sequence;
    ic -= zero_sequence;

    result.alpha = ia;
    result.beta = 0.577350f * (ib - ic);
    return result;
}

/**
 * @brief  Park 变换：静止两相 (αβ) → 旋转两相 (dq)
 * @param[in] input     αβ 轴电流分量
 * @param[in] sin_theta 电角度的正弦值
 * @param[in] cos_theta 电角度的余弦值
 * @return dq 轴电流分量
 * @note   d = cosθ·α + sinθ·β
 *         q = -sinθ·α + cosθ·β
 */
static FOC_Dq_t FOC_ParkTransform(FOC_AlphaBeta_t input, float sin_theta, float cos_theta)
{
    FOC_Dq_t result;

    result.d = cos_theta * input.alpha + sin_theta * input.beta;
    result.q = -sin_theta * input.alpha + cos_theta * input.beta;
    return result;
}

/**
 * @brief  逆 Park 变换：旋转两相 (dq) → 静止两相 (αβ)
 * @param[in] input     dq 轴电压/电流分量
 * @param[in] sin_theta 电角度的正弦值
 * @param[in] cos_theta 电角度的余弦值
 * @return αβ 轴电压/电流分量
 * @note   α = cosθ·d - sinθ·q
 *         β = sinθ·d + cosθ·q
 */
static FOC_AlphaBeta_t FOC_InverseParkTransform(FOC_Dq_t input,
                                                 float sin_theta,
                                                 float cos_theta)
{
    FOC_AlphaBeta_t result;

    result.alpha = cos_theta * input.d - sin_theta * input.q;
    result.beta = sin_theta * input.d + cos_theta * input.q;
    return result;
}

/**
 * @brief  逆 Clarke 变换：静止两相 (αβ) → 三相电压 (abc)
 * @param[in] input  αβ 轴电压分量
 * @return 三相调制电压 ua/ub/uc
 * @note   采用等幅值变换，系数 √3/2 ≈ 0.866025
 */
static ThreePhaseVoltage_t FOC_InverseClarkeTransform(FOC_AlphaBeta_t input)
{
    ThreePhaseVoltage_t result;

    result.ua = input.alpha;
    result.ub = -0.5f * input.alpha + 0.866025f * input.beta;
    result.uc = -0.5f * input.alpha - 0.866025f * input.beta;
    return result;
}

/**
 * @brief  PI 控制器单次迭代计算（含积分抗饱和）
 * @param[in]     config        PI 控制器参数（kp, ki, max_output）
 * @param[in,out] state         PI 控制器状态（积分累加值、输出值）
 * @param[in]     error         当前误差（参考值 - 实际值）
 * @param[in]     sample_period 控制周期（秒）
 * @note   输出限幅采用反算法抗饱和（back-calculation anti-windup）：
 *         当输出超过限幅值时，从积分中反向扣除本次误差贡献，
 *         避免积分饱和导致的 overshoot。
 */
static void FOC_UpdatePi(const FOC_PiConfig_t *config,
                         FOC_PiState_t *state,
                         float error,
                         float sample_period)
{
    float proportional = config->kp * error;
    float integral;

    state->integral += error * sample_period;
    integral = config->ki * state->integral;
    state->output = proportional + integral;

    if (state->output > config->max_output)
    {
        state->output = config->max_output;
        state->integral -= error * sample_period;
    }
    else if (state->output < -config->max_output)
    {
        state->output = -config->max_output;
        state->integral -= error * sample_period;
    }
}

/**
 * @brief  dq 电压矢量限幅（圆形限幅）
 * @param[in,out] voltage  dq 电压指针，限幅后原地修改
 * @param[in]     limit    电压幅值上限
 * @note   当 dq 电压矢量的模长超过 limit 时，
 *         按比例缩小至 limit，保持矢量方向不变。
 *         圆形限幅相比独立限幅能更充分地利用母线电压。
 */
static void FOC_LimitDqVoltage(FOC_Dq_t *voltage, float limit)
{
    float magnitude_squared = (voltage->d * voltage->d) + (voltage->q * voltage->q);
    float limit_squared = limit * limit;

    if (magnitude_squared > limit_squared)
    {
        float scale = limit / sqrtf(magnitude_squared);
        voltage->d *= scale;
        voltage->q *= scale;
    }
}

/**
 * @brief  复位 FOC 运行时状态（保留配置参数）
 * @note   清零所有运行时数据和参考值，但保留 PI 参数、
 *         调制限幅值等配置。应在电机停止或故障恢复时调用。
 *         调用前需确保 ADC 注入转换已停止，避免与快速环中断并发。
 */
void FOC_Reset(void)
{
    FOC_Config_t config = s_foc.config;

    memset(&s_foc, 0, sizeof(s_foc));
    s_foc.config = config;
}

/**
 * @brief  执行一次完整的 FOC 电流闭环运算
 * @param[in]  input  三相电流采样值、电角度和 dq 电流参考值
 * @param[out] output 输出的三相调制电压和实际 dq 电流（可传 NULL）
 * @note   运算流水线：
 *         abc → αβ (Clarke) → dq (Park) → PI 调节 → dq 限幅
 *         → αβ (逆 Park) → abc (逆 Clarke) → 三相调制电压
 *         由 ADC1 注入转换完成中断（20kHz）中调用，需保持高效。
 *         若 output 为 NULL，仅更新内部运行时状态。
 */
void FOC_RunCurrentLoop(const FOC_CurrentLoopInput_t *input, FOC_CurrentLoopOutput_t *output)
{
    float error_d;
    float error_q;

    if (input == NULL)
    {
        return;
    }

    s_foc.reference.d = input->id_ref;
    s_foc.reference.q = input->iq_ref;
    s_foc.runtime.ia = input->ia;
    s_foc.runtime.ib = input->ib;
    s_foc.runtime.ic = input->ic;
    s_foc.runtime.theta = input->theta;

    s_foc.runtime.current_ab = FOC_ClarkeTransform(input->ia, input->ib, input->ic);
    s_foc.runtime.sin_theta = sinf(input->theta);
    s_foc.runtime.cos_theta = cosf(input->theta);
    s_foc.runtime.current_dq = FOC_ParkTransform(s_foc.runtime.current_ab,
                                                 s_foc.runtime.sin_theta,
                                                 s_foc.runtime.cos_theta);

    error_d = s_foc.reference.d - s_foc.runtime.current_dq.d;
    error_q = s_foc.reference.q - s_foc.runtime.current_dq.q;
    FOC_UpdatePi(&s_foc.config.pi_d, &s_foc.runtime.pi_d,
                 error_d, s_foc.config.sample_period);
    FOC_UpdatePi(&s_foc.config.pi_q, &s_foc.runtime.pi_q,
                 error_q, s_foc.config.sample_period);

    s_foc.runtime.voltage_dq.d = s_foc.runtime.pi_d.output;
    s_foc.runtime.voltage_dq.q = s_foc.config.q_voltage_sign * s_foc.runtime.pi_q.output;
    FOC_LimitDqVoltage(&s_foc.runtime.voltage_dq, s_foc.config.modulation_limit);

    s_foc.runtime.voltage_ab = FOC_InverseParkTransform(s_foc.runtime.voltage_dq,
                                                        s_foc.runtime.sin_theta,
                                                        s_foc.runtime.cos_theta);
    s_foc.runtime.voltage_abc = FOC_InverseClarkeTransform(s_foc.runtime.voltage_ab);

    if (output != NULL)
    {
        output->voltage = s_foc.runtime.voltage_abc;
        output->current.d = s_foc.runtime.current_dq.d;
        output->current.q = s_foc.runtime.current_dq.q;
    }
}

/**
 * @brief  兼容接口：使用已设置的参考值执行一次 FOC 电流环
 * @param[in] ia    A 相电流（A）
 * @param[in] ib    B 相电流（A）
 * @param[in] ic    C 相电流（A）
 * @param[in] theta 电角度（rad）
 * @note   参考值由 FOC_SetCurrentReference() 预先设置，
 *         仅更新内部状态，不返回输出。
 */
void FOC_CurrentLoop(float ia, float ib, float ic, float theta)
{
    FOC_CurrentLoopInput_t input;

    input.ia = ia;
    input.ib = ib;
    input.ic = ic;
    input.theta = theta;
    input.id_ref = s_foc.reference.d;
    input.iq_ref = s_foc.reference.q;
    FOC_RunCurrentLoop(&input, NULL);
}

/**
 * @brief  设置 dq 轴电流参考值
 * @param[in] id_ref  d 轴电流参考值（A），通常为 0（最大转矩电流比）
 * @param[in] iq_ref  q 轴电流参考值（A），决定电磁转矩
 * @note   在快速环外部（如按键回调或通讯接口）中调用，
 *         设置的值将在下一次电流环迭代中生效。
 */
void FOC_SetCurrentReference(float id_ref, float iq_ref)
{
    s_foc.reference.d = id_ref;
    s_foc.reference.q = iq_ref;
}

/**
 * @brief  获取最近一次电流环计算的三相调制电压
 * @return 三相调制电压结构体（ua/ub/uc 范围与调制限幅一致）
 * @note   用于 PWM 更新或调试输出
 */
ThreePhaseVoltage_t FOC_GetThreePhaseVoltage(void)
{
    return s_foc.runtime.voltage_abc;
}

/**
 * @brief  获取最近一次电流环计算的 dq 轴实际电流
 * @return dq 轴电流（A）
 * @note   用于显示或闭环监视
 */
DQCurrent_t FOC_GetDQCurrent(void)
{
    DQCurrent_t current;

    current.d = s_foc.runtime.current_dq.d;
    current.q = s_foc.runtime.current_dq.q;
    return current;
}
