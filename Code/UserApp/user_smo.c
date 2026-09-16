/**
 * @file    user_smo.c
 * @brief   PMSM alpha-beta 坐标系滑模反电动势观测器
 * @note    输入的 PWM 占空比在本周期末写入硬件，因此本周期积分使用上次保存的电压矢量。
 */

#include "user_smo.h"
#include <math.h>
#include <string.h>

typedef struct tUsrSmoStateDef
{
    float f32CurrentAlphaHat;
    float f32CurrentBetaHat;
    float f32BemfAlpha;
    float f32BemfBeta;
    float f32VoltageAlpha;
    float f32VoltageBeta;
    float f32SlideGainV;             /* 本拍滑模注入电压上限(V)，按母线电压自动缩放 */
    float f32Theta;
    float f32Speed;
    float f32PreviousTheta;
    float f32EncoderTheta;
    float f32EncoderSpeed;
    float f32IdentResistance;
    float f32IdentInductance;
    float f32IdentBemfConstant;
    float f32IdentificationVoltageAlpha;
    float f32IdentificationVoltageBeta;
    float f32IdentLsNumerator;       /* Ls 辨识累积器：Σ(−v_d·ωe·i_q) */
    float f32IdentLsDenominator;     /* Ls 辨识累积器：Σ(ωe·i_q)² */
    uint8_t u8Valid;
} tUsrSmoStateDef;

static tUsrSmoStateDef tSmo USER_MOTOR_CCMRAM;
volatile tUserSmoScopeDef g_tUserSmoScope;

#if (USER_SMO_ENABLE != 0u)
static float UsrSmoClamp(float f32Value, float f32Min, float f32Max)
{
    if (f32Value > f32Max)
    {
        return f32Max;
    }
    if (f32Value < f32Min)
    {
        return f32Min;
    }
    return f32Value;
}
#endif

static float UsrSmoNormalizeAngle(float f32Theta)
{
    while (f32Theta >= M_2PI)
    {
        f32Theta -= M_2PI;
    }
    while (f32Theta < 0.0f)
    {
        f32Theta += M_2PI;
    }
    return f32Theta;
}

#if (USER_SMO_ENABLE != 0u)
static float UsrSmoWrapAngleDelta(float f32Delta)
{
    if (f32Delta > M_PI)
    {
        f32Delta -= M_2PI;
    }
    else if (f32Delta < -M_PI)
    {
        f32Delta += M_2PI;
    }
    return f32Delta;
}


static float UsrSmoSaturation(float f32Error)
{
    return UsrSmoClamp(f32Error / USER_SMO_BOUNDARY_LAYER_A, -1.0f, 1.0f);
}
#endif

static void UsrSmoDutyToVoltage(const tThreePhaseDutyDef *ptDuty,
                                 float f32BusVoltage,
                                 float *pf32VoltageAlpha,
                                 float *pf32VoltageBeta)
{
    float f32Ua;
    float f32Ub;
    float f32Uc;
    float f32CommonMode;

    if ((ptDuty == NULL) || (pf32VoltageAlpha == NULL) ||
        (pf32VoltageBeta == NULL))
    {
        return;
    }

    /* 先输出零电压：母线无效时提前返回也不会让调用方用到未初始化值 */
    *pf32VoltageAlpha = 0.0f;
    *pf32VoltageBeta = 0.0f;
    if (f32BusVoltage <= 0.0f)
    {
        return;
    }

    f32Ua = (((float)ptDuty->u16A / (float)USER_FOC_DUTY_Q10_MAX) - 0.5f) * f32BusVoltage;
    f32Ub = (((float)ptDuty->u16B / (float)USER_FOC_DUTY_Q10_MAX) - 0.5f) * f32BusVoltage;
    f32Uc = (((float)ptDuty->u16C / (float)USER_FOC_DUTY_Q10_MAX) - 0.5f) * f32BusVoltage;
    f32CommonMode = (f32Ua + f32Ub + f32Uc) / 3.0f;

    f32Ua -= f32CommonMode;
    f32Ub -= f32CommonMode;
    f32Uc -= f32CommonMode;
    *pf32VoltageAlpha = f32Ua;
    *pf32VoltageBeta = (f32Ub - f32Uc) * 0.577350269f;
}

void UsrSmoReset(void)
{
    memset(&tSmo, 0, sizeof(tSmo));
    memset((void *)&g_tUserSmoScope, 0, sizeof(g_tUserSmoScope));
    tSmo.f32IdentResistance = USER_SMO_PHASE_RESISTANCE_OHM;
    tSmo.f32IdentInductance = USER_SMO_PHASE_INDUCTANCE_H;
    g_tUserSmoScope.f32IdentResistance = tSmo.f32IdentResistance;
    g_tUserSmoScope.f32IdentInductance = tSmo.f32IdentInductance;
}

void UsrSmoInit(void)
{
    UsrSmoReset();
}

/**
 * @brief   更新编码器参考角与电角速度(20kHz 快环调用)
 * @param[in] f32EncoderTheta  FOC 实际使用的电角度(rad)
 * @param[in] f32EncoderSpeed  编码器电角速度(rad/s)
 * @note    只写内部状态；scope 里的编码器角/速在 1kHz 慢环与估算值同处导出，
 *          避免两者刷新率不同(20kHz vs 1kHz)导致上位机对比出假的相位差
 */
void UsrSmoSetEncoderReference(float f32EncoderTheta, float f32EncoderSpeed)
{
    tSmo.f32EncoderTheta = UsrSmoNormalizeAngle(f32EncoderTheta);
    tSmo.f32EncoderSpeed = f32EncoderSpeed;
}

static void UsrSmoUpdateIdentification(float f32CurrentAlpha,
                                       float f32CurrentBeta)
{
#if (USER_SMO_ENABLE_IDENTIFICATION != 0u)
    /* 辨识在 1kHz 慢环调用。分步辨识：Ke 由 SMO 反电动势幅值/电角速度求得(仅作输出)，
       Ls 用 d 轴电压方程求解，与 Ke/BEMF 完全解耦，避免三参数联合辨识的秩亏损
       以及反电动势相位误差对 Ls 的污染。 */
    const float f32Forget = 1.0f - USER_SMO_IDENT_LPF_ALPHA;
    float f32CurrentMagnitude;
    float f32SpeedMagnitude;
    float f32KeInstant;
    float f32BemfMagnitude;
    float f32Vd;
    float f32Iq;
    float f32Cross;

    f32CurrentMagnitude = sqrtf((f32CurrentAlpha * f32CurrentAlpha) +
                                 (f32CurrentBeta * f32CurrentBeta));
    f32SpeedMagnitude = fabsf(tSmo.f32EncoderSpeed);
    /* 导出辨识门槛的实际输入，供调试器/上位机判断辨识为何未更新 */
    g_tUserSmoScope.f32IdentCurrentMagnitude = f32CurrentMagnitude;
    g_tUserSmoScope.f32IdentSpeedMagnitude = f32SpeedMagnitude;

    /* ---- 第 1 步：Ke 辨识(仅作输出，供后续整定滑模增益使用) ----
       e_αβ = Ke·ωe·[-sinθe, cosθe]，故 Ke = |e_αβ|/|ωe|。
       用 SMO 估计反电动势幅值，转速足够且幅值可信时更新，与电流大小无关。 */
    f32BemfMagnitude = sqrtf((tSmo.f32BemfAlpha * tSmo.f32BemfAlpha) +
                             (tSmo.f32BemfBeta * tSmo.f32BemfBeta));
    if ((f32SpeedMagnitude >= USER_SMO_IDENT_SPEED_MIN_RAD_S) &&
        (f32BemfMagnitude >= USER_SMO_BEMF_VALID_MIN_V))
    {
        f32KeInstant = f32BemfMagnitude / f32SpeedMagnitude;
        tSmo.f32IdentBemfConstant +=
            USER_SMO_IDENT_KE_LPF_ALPHA * (f32KeInstant - tSmo.f32IdentBemfConstant);
    }

    /* ---- 第 2 步：Ls 辨识(d 轴电压方程，与 Ke/BEMF 完全解耦) ----
       d 轴方程：v_d = Rs·i_d + Ls·di_d/dt − ωe·Ls·i_q。
       FOC 保持 Id≈0 且稳态时：v_d ≈ −ωe·Ls·i_q → Ls = −v_d/(ωe·i_q)。
       d 轴无反电动势项，故不再依赖 Ke；累积投影消噪：
       Ls = Σ(−v_d·ωe·i_q) / Σ(ωe·i_q)²，需要 |ωe·i_q| 足够大避免除噪声。 */
    f32Vd = (tSmo.f32IdentificationVoltageAlpha * cosf(tSmo.f32EncoderTheta)) +
            (tSmo.f32IdentificationVoltageBeta * sinf(tSmo.f32EncoderTheta));
    f32Iq = (-f32CurrentAlpha * sinf(tSmo.f32EncoderTheta)) +
            (f32CurrentBeta * cosf(tSmo.f32EncoderTheta));
    f32Cross = tSmo.f32EncoderSpeed * f32Iq;   /* ωe·i_q */
    /* 导出辨识中间量，供调试器/上位机核对符号与量级 */
    g_tUserSmoScope.f32IdentVd = f32Vd;
    g_tUserSmoScope.f32IdentIq = f32Iq;
    g_tUserSmoScope.f32IdentCross = f32Cross;
    if ((f32CurrentMagnitude >= USER_SMO_IDENT_CURRENT_MIN_A) &&
        (f32SpeedMagnitude >= USER_SMO_IDENT_SPEED_MIN_RAD_S) &&
        (fabsf(f32Cross) >= USER_SMO_IDENT_LS_CROSS_MIN))
    {
        tSmo.f32IdentLsNumerator = (f32Forget * tSmo.f32IdentLsNumerator) +
                                   (USER_SMO_IDENT_LPF_ALPHA * ((-f32Vd) * f32Cross));
        tSmo.f32IdentLsDenominator = (f32Forget * tSmo.f32IdentLsDenominator) +
                                     (USER_SMO_IDENT_LPF_ALPHA * (f32Cross * f32Cross));
        if (tSmo.f32IdentLsDenominator > 1e-9f)
        {
            g_tUserSmoScope.f32IdentLsRaw = tSmo.f32IdentLsNumerator / tSmo.f32IdentLsDenominator;
            tSmo.f32IdentInductance = g_tUserSmoScope.f32IdentLsRaw;
            if (tSmo.f32IdentInductance < 0.0001f)
            {
                tSmo.f32IdentInductance = 0.0001f;   /* 防止物理不可能的过小/负值 */
            }
        }
    }
#else
    (void)f32CurrentAlpha;
    (void)f32CurrentBeta;
#endif
}

USER_MOTOR_FAST_CODE void UsrSmoFastUpdate(float f32CurrentAlpha,
                                             float f32CurrentBeta,
                                             const tThreePhaseDutyDef *ptNextDuty,
                                             float f32BusVoltage)
{
    /* 当前 ADC 电流样本对应上一个 PWM 周期施加的电压。 */
    tSmo.f32IdentificationVoltageAlpha = tSmo.f32VoltageAlpha;
    tSmo.f32IdentificationVoltageBeta = tSmo.f32VoltageBeta;

#if (USER_SMO_ENABLE != 0u)
    const float f32Ts = 1.0f / USER_SMO_LOOP_HZ;
    float f32NextVoltageAlpha = 0.0f;
    float f32NextVoltageBeta = 0.0f;
    float f32SlideAlpha;
    float f32SlideBeta;

    /* 滑模注入上限必须大于最高转速下的反电动势幅值：反电动势不会超过可输出电压
       Vbus/√3，故按母线电压比例设定，转速提升时自动放大。 */
    tSmo.f32SlideGainV = USER_SMO_SLIDE_GAIN_RATIO * f32BusVoltage * 0.577350269f;

    f32SlideAlpha = tSmo.f32SlideGainV *
                    UsrSmoSaturation(tSmo.f32CurrentAlphaHat - f32CurrentAlpha);
    f32SlideBeta = tSmo.f32SlideGainV *
                   UsrSmoSaturation(tSmo.f32CurrentBetaHat - f32CurrentBeta);

    tSmo.f32CurrentAlphaHat += f32Ts *
        ((tSmo.f32VoltageAlpha - (USER_SMO_PHASE_RESISTANCE_OHM * tSmo.f32CurrentAlphaHat) - f32SlideAlpha) /
         USER_SMO_PHASE_INDUCTANCE_H);
    tSmo.f32CurrentBetaHat += f32Ts *
        ((tSmo.f32VoltageBeta - (USER_SMO_PHASE_RESISTANCE_OHM * tSmo.f32CurrentBetaHat) - f32SlideBeta) /
         USER_SMO_PHASE_INDUCTANCE_H);

    tSmo.f32BemfAlpha += USER_SMO_BEMF_FAST_FILTER_ALPHA * (f32SlideAlpha - tSmo.f32BemfAlpha);
    tSmo.f32BemfBeta += USER_SMO_BEMF_FAST_FILTER_ALPHA * (f32SlideBeta - tSmo.f32BemfBeta);

    UsrSmoDutyToVoltage(ptNextDuty, f32BusVoltage,
                        &f32NextVoltageAlpha, &f32NextVoltageBeta);
    tSmo.f32VoltageAlpha = f32NextVoltageAlpha;
    tSmo.f32VoltageBeta = f32NextVoltageBeta;

    /* 影子运行观测：导出实测/模型电流、电流误差与最近一次电压，供 Logic Analyzer 对比；
       同时供 1kHz 慢环的在线辨识读取实测电流。 */
    g_tUserSmoScope.f32CurrentAlpha = f32CurrentAlpha;
    g_tUserSmoScope.f32CurrentBeta = f32CurrentBeta;
    g_tUserSmoScope.f32CurrentAlphaHat = tSmo.f32CurrentAlphaHat;
    g_tUserSmoScope.f32CurrentBetaHat = tSmo.f32CurrentBetaHat;
    g_tUserSmoScope.f32CurrentErrorAlpha = tSmo.f32CurrentAlphaHat - f32CurrentAlpha;
    g_tUserSmoScope.f32CurrentErrorBeta = tSmo.f32CurrentBetaHat - f32CurrentBeta;
    g_tUserSmoScope.f32VoltageAlpha = tSmo.f32VoltageAlpha;
    g_tUserSmoScope.f32VoltageBeta = tSmo.f32VoltageBeta;
#else
    (void)f32CurrentAlpha;
    (void)f32CurrentBeta;
    (void)ptNextDuty;
    (void)f32BusVoltage;
#endif
}

void UsrSmoSlowUpdate(void)
{
#if (USER_SMO_ENABLE != 0u)
    float f32BemfMagnitude;
    float f32Angle;
    float f32DeltaTheta;
    float f32Speed;

    UsrSmoUpdateIdentification(g_tUserSmoScope.f32CurrentAlpha,
                               g_tUserSmoScope.f32CurrentBeta);
    f32BemfMagnitude = sqrtf((tSmo.f32BemfAlpha * tSmo.f32BemfAlpha) +
                             (tSmo.f32BemfBeta * tSmo.f32BemfBeta));
    tSmo.u8Valid = (f32BemfMagnitude >= USER_SMO_BEMF_VALID_MIN_V) ? 1u : 0u;
    f32Angle = UsrSmoNormalizeAngle(atan2f(tSmo.f32BemfBeta, tSmo.f32BemfAlpha) - (M_PI * 0.5f));
    if (tSmo.u8Valid == 0u)
    {
        tSmo.f32PreviousTheta = f32Angle;
        tSmo.f32Speed = 0.0f;
    }
    else
    {
        f32DeltaTheta = UsrSmoWrapAngleDelta(f32Angle - tSmo.f32PreviousTheta);
        f32Speed = f32DeltaTheta * 1000.0f;
        tSmo.f32Speed += USER_SMO_SPEED_FILTER_ALPHA * (f32Speed - tSmo.f32Speed);
    }
    tSmo.f32Theta = f32Angle;
    tSmo.f32PreviousTheta = f32Angle;
#endif

    /* 编码器角/速与估算角在同一处导出(1kHz)，保证上位机读到的是同一时刻的一对 */
    g_tUserSmoScope.f32EncoderTheta = tSmo.f32EncoderTheta;
    g_tUserSmoScope.f32EncoderSpeed = tSmo.f32EncoderSpeed;
    g_tUserSmoScope.f32EstimatedTheta = tSmo.f32Theta;
    g_tUserSmoScope.f32EstimatedSpeed = tSmo.f32Speed;
    g_tUserSmoScope.f32BemfAlpha = tSmo.f32BemfAlpha;
    g_tUserSmoScope.f32BemfBeta = tSmo.f32BemfBeta;
    g_tUserSmoScope.f32IdentResistance = tSmo.f32IdentResistance;
    g_tUserSmoScope.f32IdentInductance = tSmo.f32IdentInductance;
    g_tUserSmoScope.f32IdentBemfConstant = tSmo.f32IdentBemfConstant;
    g_tUserSmoScope.u8Valid = tSmo.u8Valid;
}

float UsrSmoGetElectricalAngle(void)
{
    return tSmo.f32Theta;
}

float UsrSmoGetElectricalSpeed(void)
{
    return tSmo.f32Speed;
}

uint8_t UsrSmoIsValid(void)
{
    return tSmo.u8Valid;
}
