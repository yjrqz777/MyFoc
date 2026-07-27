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
#define FOC_CONTROL_TS              (0.00005f)

/** @brief dq 电压调制限幅值 */
#define FOC_MODULATION_LIMIT        (20.0f)

/** @brief q 轴电压符号（1.0 = 正方向） */
#define FOC_Q_VOLTAGE_SIGN          (1.0f)

/** @brief 静止坐标系电流（αβ 轴） */
typedef struct tFocAlphaBetaDef
{
    float f32Alpha;  /**< α 轴电流分量 */
    float f32Beta;   /**< β 轴电流分量 */
} tFocAlphaBetaDef;

/** @brief 同步旋转坐标系电压/电流（dq 轴） */
typedef struct tFocDqDef
{
    float f32D;  /**< d 轴分量（励磁） */
    float f32Q;  /**< q 轴分量（转矩） */
} tFocDqDef;

/** @brief PI 控制器配置参数 */
typedef struct tFocPiConfigDef
{
    float f32Kp;           /**< 比例增益 */
    float f32Ki;           /**< 积分增益 */
    float f32MaxOutput;    /**< 输出限幅值 */
} tFocPiConfigDef;

/** @brief PI 控制器运行时状态 */
typedef struct tFocPiStateDef
{
    float f32Integral;  /**< 积分累加值 */
    float f32Output;    /**< 当前输出值 */
} tFocPiStateDef;

/** @brief FOC 全局配置参数 */
typedef struct tFocConfigDef
{
    float f32SamplePeriod;        /**< 控制周期（秒） */
    float f32ModulationLimit;     /**< 调制电压限幅值 */
    float f32QVoltageSign;        /**< q 轴电压方向符号 */
    tFocPiConfigDef tPiD;         /**< d 轴 PI 控制器参数 */
    tFocPiConfigDef tPiQ;         /**< q 轴 PI 控制器参数 */
} tFocConfigDef;

/** @brief dq 电流参考值 */
typedef struct tFocReferenceDef
{
    float f32D;  /**< d 轴电流参考值（A），通常 Id_ref = 0 */
    float f32Q;  /**< q 轴电流参考值（A），决定转矩输出 */
} tFocReferenceDef;

/** @brief FOC 单次运算运行时数据（所有中间变量） */
typedef struct tFocRuntimeDef
{
    float f32Ia;                       /**< A 相采样电流（A） */
    float f32Ib;                       /**< B 相采样电流（A） */
    float f32Ic;                       /**< C 相采样电流（A） */
    float f32Theta;                    /**< 电角度（rad） */
    float f32SinTheta;                 /**< 电角度的正弦值 */
    float f32CosTheta;                 /**< 电角度的余弦值 */
    tFocAlphaBetaDef tCurrentAb;       /**< αβ 轴电流（Clarke 变换后） */
    tFocDqDef tCurrentDq;              /**< dq 轴电流（Park 变换后） */
    tFocPiStateDef tPiD;               /**< d 轴 PI 控制器运行时状态 */
    tFocPiStateDef tPiQ;               /**< q 轴 PI 控制器运行时状态 */
    tFocDqDef tVoltageDq;              /**< dq 轴电压（PI 输出） */
    tFocAlphaBetaDef tVoltageAb;       /**< αβ 轴电压（逆 Park 变换后） */
    tThreePhaseVoltageDef tVoltageAbc; /**< 三相调制电压（逆 Clarke 变换后） */
} tFocRuntimeDef;

/** @brief FOC 全局上下文（配置 + 参考值 + 运行时） */
typedef struct tFocContextDef
{
    tFocConfigDef tConfig;       /**< 控制参数（不可在快速环中修改） */
    tFocReferenceDef tReference; /**< dq 电流参考值 */
    tFocRuntimeDef tRuntime;     /**< 运行时数据，每次迭代更新 */
} tFocContextDef;

static tFocContextDef tFoc = {
    .tConfig = {
        .f32SamplePeriod = FOC_CONTROL_TS,
        .f32ModulationLimit = FOC_MODULATION_LIMIT,
        .f32QVoltageSign = FOC_Q_VOLTAGE_SIGN,
        .tPiD = {
            .f32Kp = 0.35f,
            .f32Ki = 2.0f,
            .f32MaxOutput = FOC_MODULATION_LIMIT
        },
        .tPiQ = {
            .f32Kp = 0.5f,
            .f32Ki = 1.5f,
            .f32MaxOutput = FOC_MODULATION_LIMIT
        }
    },
    .tReference = {0.0f, 0.0f}
};

/**
 * @brief  Clarke 变换：三相电流 (abc) → 静止两相 (αβ)
 * @param[in] f32Ia  A 相电流（A）
 * @param[in] f32Ib  B 相电流（A）
 * @param[in] f32Ic  C 相电流（A）
 * @return αβ 轴电流分量
 * @note   采用等幅值变换，β 系数为 1/√3 ≈ 0.57735
 */
static tFocAlphaBetaDef UsrFocClarkeTransform(float f32Ia, float f32Ib, float f32Ic)
{
    tFocAlphaBetaDef Result;
    float ZeroSequence = (f32Ia + f32Ib + f32Ic) * 0.333333f;

    /* Three motor phase currents should sum to zero.  Remove common-mode
     * sampling error before Clarke transform; fixed-vector tests show a
     * noticeable Ia+Ib+Ic offset caused by the low-side sampling window.
     */
    f32Ia -= ZeroSequence;
    f32Ib -= ZeroSequence;
    f32Ic -= ZeroSequence;

    Result.f32Alpha = f32Ia;
    Result.f32Beta = 0.577350f * (f32Ib - f32Ic);
    return Result;
}

/**
 * @brief  Park 变换：静止两相 (αβ) → 旋转两相 (dq)
 * @param[in] tInput     αβ 轴电流分量
 * @param[in] f32SinTheta 电角度的正弦值
 * @param[in] f32CosTheta 电角度的余弦值
 * @return dq 轴电流分量
 * @note   d = cosθ·α + sinθ·β
 *         q = -sinθ·α + cosθ·β
 */
static tFocDqDef UsrFocParkTransform(tFocAlphaBetaDef tInput, float f32SinTheta, float f32CosTheta)
{
    tFocDqDef Result;

    Result.f32D = f32CosTheta * tInput.f32Alpha + f32SinTheta * tInput.f32Beta;
    Result.f32Q = -f32SinTheta * tInput.f32Alpha + f32CosTheta * tInput.f32Beta;
    return Result;
}

/**
 * @brief  逆 Park 变换：旋转两相 (dq) → 静止两相 (αβ)
 * @param[in] tInput     dq 轴电压/电流分量
 * @param[in] f32SinTheta 电角度的正弦值
 * @param[in] f32CosTheta 电角度的余弦值
 * @return αβ 轴电压/电流分量
 * @note   α = cosθ·d - sinθ·q
 *         β = sinθ·d + cosθ·q
 */
static tFocAlphaBetaDef UsrFocInverseParkTransform(tFocDqDef tInput,
                                                   float f32SinTheta,
                                                   float f32CosTheta)
{
    tFocAlphaBetaDef Result;

    Result.f32Alpha = f32CosTheta * tInput.f32D - f32SinTheta * tInput.f32Q;
    Result.f32Beta = f32SinTheta * tInput.f32D + f32CosTheta * tInput.f32Q;
    return Result;
}

/**
 * @brief  逆 Clarke 变换：静止两相 (αβ) → 三相电压 (abc)
 * @param[in] tInput  αβ 轴电压分量
 * @return 三相调制电压 ua/ub/uc
 * @note   采用等幅值变换，系数 √3/2 ≈ 0.866025
 */
static tThreePhaseVoltageDef UsrFocInverseClarkeTransform(tFocAlphaBetaDef tInput)
{
    tThreePhaseVoltageDef Result;

    Result.f32Ua = tInput.f32Alpha;
    Result.f32Ub = -0.5f * tInput.f32Alpha + 0.866025f * tInput.f32Beta;
    Result.f32Uc = -0.5f * tInput.f32Alpha - 0.866025f * tInput.f32Beta;
    return Result;
}

/**
 * @brief  PI 控制器单次迭代计算（含积分抗饱和）
 * @param[in]     ptConfig        PI 控制器参数（kp, ki, max_output）
 * @param[in,out] ptState         PI 控制器状态（积分累加值、输出值）
 * @param[in]     f32Error         当前误差（参考值 - 实际值）
 * @param[in]     f32SamplePeriod 控制周期（秒）
 * @note   输出限幅采用反算法抗饱和（back-calculation anti-windup）：
 *         当输出超过限幅值时，从积分中反向扣除本次误差贡献，
 *         避免积分饱和导致的 overshoot。
 */
static void UsrFocUpdatePi(const tFocPiConfigDef * ptConfig,
                           tFocPiStateDef * ptState,
                           float f32Error,
                           float f32SamplePeriod)
{
    float Proportional = ptConfig->f32Kp * f32Error;
    float Integral;

    ptState->f32Integral += f32Error * f32SamplePeriod;
    Integral = ptConfig->f32Ki * ptState->f32Integral;
    ptState->f32Output = Proportional + Integral;

    if (ptState->f32Output > ptConfig->f32MaxOutput)
    {
        ptState->f32Output = ptConfig->f32MaxOutput;
        ptState->f32Integral -= f32Error * f32SamplePeriod;
    }
    else if (ptState->f32Output < -ptConfig->f32MaxOutput)
    {
        ptState->f32Output = -ptConfig->f32MaxOutput;
        ptState->f32Integral -= f32Error * f32SamplePeriod;
    }
}

/**
 * @brief  dq 电压矢量限幅（圆形限幅）
 * @param[in,out] ptVoltage  dq 电压指针，限幅后原地修改
 * @param[in]     f32Limit    电压幅值上限
 * @note   当 dq 电压矢量的模长超过 limit 时，
 *         按比例缩小至 limit，保持矢量方向不变。
 *         圆形限幅相比独立限幅能更充分地利用母线电压。
 */
static void UsrFocLimitDqVoltage(tFocDqDef * ptVoltage, float f32Limit)
{
    float MagnitudeSquared = (ptVoltage->f32D * ptVoltage->f32D) + (ptVoltage->f32Q * ptVoltage->f32Q);
    float LimitSquared = f32Limit * f32Limit;

    if (MagnitudeSquared > LimitSquared)
    {
        float Scale = f32Limit / sqrtf(MagnitudeSquared);
        ptVoltage->f32D *= Scale;
        ptVoltage->f32Q *= Scale;
    }
}

/**
 * @brief  复位 FOC 运行时状态（保留配置参数）
 * @note   清零所有运行时数据和参考值，但保留 PI 参数、
 *         调制限幅值等配置。应在电机停止或故障恢复时调用。
 *         调用前需确保 ADC 注入转换已停止，避免与快速环中断并发。
 */
void UsrFocReset(void)
{
    tFocConfigDef Config = tFoc.tConfig;

    memset(&tFoc, 0, sizeof(tFoc));
    tFoc.tConfig = Config;
}

/**
 * @brief  执行一次完整的 FOC 电流闭环运算
 * @param[in]  ptInput  三相电流采样值、电角度和 dq 电流参考值
 * @param[out] ptOutput 输出的三相调制电压和实际 dq 电流（可传 NULL）
 * @note   运算流水线：
 *         abc → αβ (Clarke) → dq (Park) → PI 调节 → dq 限幅
 *         → αβ (逆 Park) → abc (逆 Clarke) → 三相调制电压
 *         由 ADC1 注入转换完成中断（20kHz）中调用，需保持高效。
 *         若 output 为 NULL，仅更新内部运行时状态。
 */
void UsrFocRunCurrentLoop(const tFocCurrentLoopInputDef * ptInput, tFocCurrentLoopOutputDef * ptOutput)
{
    float ErrorD;
    float ErrorQ;

    if (ptInput == NULL)
    {
        return;
    }

    tFoc.tReference.f32D = ptInput->f32IdReference;
    tFoc.tReference.f32Q = ptInput->f32IqReference;
    tFoc.tRuntime.f32Ia = ptInput->f32Ia;
    tFoc.tRuntime.f32Ib = ptInput->f32Ib;
    tFoc.tRuntime.f32Ic = ptInput->f32Ic;
    tFoc.tRuntime.f32Theta = ptInput->f32Theta;

    tFoc.tRuntime.tCurrentAb = UsrFocClarkeTransform(ptInput->f32Ia, ptInput->f32Ib, ptInput->f32Ic);
    tFoc.tRuntime.f32SinTheta = sinf(ptInput->f32Theta);
    tFoc.tRuntime.f32CosTheta = cosf(ptInput->f32Theta);
    tFoc.tRuntime.tCurrentDq = UsrFocParkTransform(tFoc.tRuntime.tCurrentAb,
                                                  tFoc.tRuntime.f32SinTheta,
                                                  tFoc.tRuntime.f32CosTheta);

    ErrorD = tFoc.tReference.f32D - tFoc.tRuntime.tCurrentDq.f32D;
    ErrorQ = tFoc.tReference.f32Q - tFoc.tRuntime.tCurrentDq.f32Q;
    UsrFocUpdatePi(&tFoc.tConfig.tPiD, &tFoc.tRuntime.tPiD,
                   ErrorD, tFoc.tConfig.f32SamplePeriod);
    UsrFocUpdatePi(&tFoc.tConfig.tPiQ, &tFoc.tRuntime.tPiQ,
                   ErrorQ, tFoc.tConfig.f32SamplePeriod);

    tFoc.tRuntime.tVoltageDq.f32D = tFoc.tRuntime.tPiD.f32Output;
    tFoc.tRuntime.tVoltageDq.f32Q = tFoc.tConfig.f32QVoltageSign * tFoc.tRuntime.tPiQ.f32Output;
    UsrFocLimitDqVoltage(&tFoc.tRuntime.tVoltageDq, tFoc.tConfig.f32ModulationLimit);

    tFoc.tRuntime.tVoltageAb = UsrFocInverseParkTransform(tFoc.tRuntime.tVoltageDq,
                                                         tFoc.tRuntime.f32SinTheta,
                                                         tFoc.tRuntime.f32CosTheta);
    tFoc.tRuntime.tVoltageAbc = UsrFocInverseClarkeTransform(tFoc.tRuntime.tVoltageAb);

    if (ptOutput != NULL)
    {
        ptOutput->tVoltage = tFoc.tRuntime.tVoltageAbc;
        ptOutput->tCurrent.f32D = tFoc.tRuntime.tCurrentDq.f32D;
        ptOutput->tCurrent.f32Q = tFoc.tRuntime.tCurrentDq.f32Q;
    }
}

/**
 * @brief  兼容接口：使用已设置的参考值执行一次 FOC 电流环
 * @param[in] f32Ia    A 相电流（A）
 * @param[in] f32Ib    B 相电流（A）
 * @param[in] f32Ic    C 相电流（A）
 * @param[in] f32Theta 电角度（rad）
 * @note   参考值由 UsrFocSetCurrentReference() 预先设置，
 *         仅更新内部状态，不返回输出。
 */
void UsrFocCurrentLoop(float f32Ia, float f32Ib, float f32Ic, float f32Theta)
{
    tFocCurrentLoopInputDef Input;

    Input.f32Ia = f32Ia;
    Input.f32Ib = f32Ib;
    Input.f32Ic = f32Ic;
    Input.f32Theta = f32Theta;
    Input.f32IdReference = tFoc.tReference.f32D;
    Input.f32IqReference = tFoc.tReference.f32Q;
    UsrFocRunCurrentLoop(&Input, NULL);
}

/**
 * @brief  设置 dq 轴电流参考值
 * @param[in] f32IdReference  d 轴电流参考值（A），通常为 0（最大转矩电流比）
 * @param[in] f32IqReference  q 轴电流参考值（A），决定电磁转矩
 * @note   在快速环外部（如按键回调或通讯接口）中调用，
 *         设置的值将在下一次电流环迭代中生效。
 */
void UsrFocSetCurrentReference(float f32IdReference, float f32IqReference)
{
    tFoc.tReference.f32D = f32IdReference;
    tFoc.tReference.f32Q = f32IqReference;
}

/**
 * @brief  获取最近一次电流环计算的三相调制电压
 * @return 三相调制电压结构体（ua/ub/uc 范围与调制限幅一致）
 * @note   用于 PWM 更新或调试输出
 */
tThreePhaseVoltageDef UsrFocGetThreePhaseVoltage(void)
{
    return tFoc.tRuntime.tVoltageAbc;
}

/**
 * @brief  获取最近一次电流环计算的 dq 轴实际电流
 * @return dq 轴电流（A）
 * @note   用于显示或闭环监视
 */
tDqCurrentDef UsrFocGetDqCurrent(void)
{
    tDqCurrentDef Current;

    Current.f32D = tFoc.tRuntime.tCurrentDq.f32D;
    Current.f32Q = tFoc.tRuntime.tCurrentDq.f32Q;
    return Current;
}
