/**
 * @file    user_smo.h
 * @brief   PMSM 滑模反电动势观测器接口
 * @note    当前仅用于与 MT6816 编码器并行的影子运行，不参与 FOC Park 角度或速度环反馈。
 */

#ifndef __USER_SMO_H__
#define __USER_SMO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_foc.h"

/*
 * SMO 影子开关：只在后台估算，Park 角度和速度环始终保持编码器反馈。
 * 初始 Rs/Ls 仅用于让模型可运行；影子过程会输出在线辨识值供后续回填。
 */
#define USER_SMO_ENABLE                    (1u)
#define USER_SMO_PHASE_RESISTANCE_OHM      (2.72f)    /* Rs 相电阻：线间实测 5.447/2(星形接法两相串联) */
#define USER_SMO_PHASE_INDUCTANCE_H        (0.002f)   /* Ls 实测：堵转法 ~2mH(Ls1≈1.7mH/Ls2≈2.1mH)，误差±30% */
#define USER_SMO_ENABLE_IDENTIFICATION     (1u)

/* 观测器和在线辨识参数，首次运行后基于编码器影子波形再整定。 */
#define USER_SMO_LOOP_HZ                   (20000.0f)   /* 快环速率：电流模型积分 50us */
#define USER_SMO_SLOW_LOOP_HZ              (1000.0f)    /* 慢环速率：辨识 di/dt 按 1ms 采样间隔计算 */
/* 滑模注入电压上限 = RATIO × Vbus/√3：反电动势幅值不会超过可输出电压(Vbus/√3)，
   故按母线电压比例自动缩放，转速提升时自动放大，保证任何转速下滑模面都能压住误差。
   若已知 Ke(V/(rad/s)) 与最高电角速度 ω_e，也可改用固定值 K = 1.5~2 × Ke × ω_e。 */
#define USER_SMO_SLIDE_GAIN_RATIO          (0.5f)     /* k≈0.5×Vbus/√3；Ls=5mH 时边界层增益 0.2/步，稳定且覆盖中高速 BEMF */
#define USER_SMO_BOUNDARY_LAYER_A          (0.30f)  /* 饱和函数边界层(A) */
#define USER_SMO_BEMF_FAST_FILTER_ALPHA    (0.12f)  /* 20kHz 反电动势 IIR 系数 */
#define USER_SMO_SPEED_FILTER_ALPHA        (0.10f)
#define USER_SMO_BEMF_VALID_MIN_V          (0.20f)
#define USER_SMO_IDENT_CURRENT_MIN_A       (0.20f)  /* 电流幅值低于此值不辨识 Ls，避免除噪声 */
#define USER_SMO_IDENT_SPEED_MIN_RAD_S     (80.0f)  /* 电角速度过低时不辨识，避免反电动势过小 */
#define USER_SMO_IDENT_LPF_ALPHA           (0.002f) /* Ls 辨识累积低通系数(1kHz 慢环) */
#define USER_SMO_IDENT_KE_LPF_ALPHA        (0.02f)  /* Ke 辨识滤波系数(1kHz 慢环) */
#define USER_SMO_IDENT_LS_CROSS_MIN        (50.0f)  /* |ωe·i_q| 低于此值不辨识 Ls，避免除噪声 */

typedef struct tUserSmoScopeDef
{
    float f32EncoderTheta;       /* 编码器电角度(rad)，仅用于影子运行对比 */
    float f32EstimatedTheta;     /* SMO 估算电角度(rad) */
    float f32EstimatedSpeed;     /* SMO 估算电角速度(rad/s) */
    float f32CurrentAlpha;       /* 实测 alpha 轴电流(A) */
    float f32CurrentBeta;        /* 实测 beta 轴电流(A) */
    float f32CurrentAlphaHat;    /* SMO 模型 alpha 轴估算电流(A) */
    float f32CurrentBetaHat;     /* SMO 模型 beta 轴估算电流(A) */
    float f32CurrentErrorAlpha;  /* alpha 轴电流误差：估算-实测(A) */
    float f32CurrentErrorBeta;   /* beta 轴电流误差：估算-实测(A) */
    float f32BemfAlpha;          /* 估算 alpha 轴反电动势(V) */
    float f32BemfBeta;           /* 估算 beta 轴反电动势(V) */
    float f32VoltageAlpha;       /* 上一 PWM 周期的 alpha 轴电压(V) */
    float f32VoltageBeta;        /* 上一 PWM 周期的 beta 轴电压(V) */
    float f32IdentResistance;    /* 编码器影子辨识的 Rs 估算值(Ohm) */
    float f32IdentInductance;    /* 编码器影子辨识的 Ls 估算值(H) */
    float f32IdentBemfConstant;  /* 反电动势常数估算值(V/(rad/s)) */
    float f32IdentCurrentMagnitude;  /* 辨识门槛用：实测电流幅值 sqrt(iα²+iβ²)(A) */
    float f32IdentSpeedMagnitude;    /* 辨识门槛用：编码器电角速度绝对值(rad/s) */
    float f32IdentVd;            /* 辨识中间量：d 轴电压(V) */
    float f32IdentIq;            /* 辨识中间量：q 轴电流(A) */
    float f32IdentCross;         /* 辨识中间量：ωe·i_q(rad/s·A) */
    float f32IdentLsRaw;         /* 辨识原始值：钳位前 Ls(H)，为负即符号/约定问题 */
    float f32EncoderSpeed;       /* 编码器电角速度(rad/s) */
    uint8_t u8Valid;             /* 反电动势幅值达到有效阈值 */
} tUserSmoScopeDef;

void UsrSmoInit(void);
void UsrSmoReset(void);
void UsrSmoFastUpdate(float f32CurrentAlpha,
                      float f32CurrentBeta,
                      const tThreePhaseDutyDef *ptNextDuty,
                      float f32BusVoltage);
void UsrSmoSlowUpdate(void);
void UsrSmoSetEncoderReference(float f32EncoderTheta, float f32EncoderSpeed);
float UsrSmoGetElectricalAngle(void);
float UsrSmoGetElectricalSpeed(void);
uint8_t UsrSmoIsValid(void);

extern volatile tUserSmoScopeDef g_tUserSmoScope;

#ifdef __cplusplus
}
#endif

#endif /* __USER_SMO_H__ */
