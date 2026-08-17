#ifndef __USER_FOC_H__
#define __USER_FOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

#define USER_FOC_DUTY_Q10_MAX (1024u)


#define USER_FOC_PI_KP_Q16                (-20000)   /* 电流环PI比例系数(Q16) */
#define USER_FOC_PI_KI_Q16                (-100)      /* 电流环PI积分系数(Q16) */
#define USER_FOC_PI_KD_Q16                (0)       /* 电流环PI微分系数(Q16)，未启用 */
#define USER_FOC_PI_OUTPUT_MAX            (1000)     /* PI输出电压上限(Q10) */
#define USER_FOC_PI_OUTPUT_MIN            (-1000)    /* PI输出电压下限(Q10) */
#define USER_FOC_CURRENT_LOOP_KP_STEP_Q16 (500)     /* 电流环Kp按键调整步进(Q16) */
#define USER_FOC_CURRENT_LOOP_KI_STEP_Q16 (10)      /* 电流环Ki按键调整步进(Q16) */


#define USER_FOC_MODULATION_LIMIT_Q10     (591)     /* SVPWM调制比限幅(Q10) */

#define USER_FOC_CURRENT_FILTER_ALPHA_Q15 (2000)    /* 电流采样一阶滤波系数(Q15)，截止~95Hz，滞后~1.6ms */
#define USER_FOC_DUTY_FILTER_ALPHA_Q15    (3564)    /* 占空比一阶滤波系数(Q15) — 恢复原始值 */

#define USER_FOC_FILTER_Q_SHIFT           (15)      /* 滤波状态定点Q格式移位 */



#define USER_FOC_CURRENT_AMPS_PER_CODE    (3.3f / 4096.0f / 0.010f / 30.0f)   /* 电流码值→安培换算系数(A/LSB) */

typedef struct tThreePhaseDutyDef
{
    uint16_t u16A;
    uint16_t u16B;
    uint16_t u16C;
} tThreePhaseDutyDef;

typedef struct tDqCurrentDef
{
    float f32D;
    float f32Q;
} tDqCurrentDef;

typedef struct tFocInputDef
{
    float f32Ia;
    float f32Ib;
    float f32Ic;
    float f32Theta;
    float f32IdRef;
    float f32IqRef;
} tFocInputDef;

typedef struct tFocOutputDef
{
    tThreePhaseDutyDef tDuty;
    tDqCurrentDef tCurrent;
} tFocOutputDef;

/* Keil Logic Analyzer 电流环观测数据，单位均为 A。 */
typedef struct tUserFocScopeDef
{
    float f32IqRef;
    float f32IqFbk;
    float f32IdRef;
    float f32IdFbk;
} tUserFocScopeDef;

void UsrFocReset(void);
void UsrFocCurrentLoop(const tFocInputDef *ptInput,
                          tFocOutputDef *ptOutput);
tDqCurrentDef UsrFocGetDqCurrent(void);

/* 电流环 Kp/Ki 运行时调参接口（Q16 定点） */
void UsrFocAdjustCurrentLoopKp(int32_t s32DeltaQ16);
void UsrFocAdjustCurrentLoopKi(int32_t s32DeltaQ16);
int32_t UsrFocGetCurrentLoopKpQ16(void);
int32_t UsrFocGetCurrentLoopKiQ16(void);

extern volatile tUserFocScopeDef g_tUserFocScope;

#ifdef __cplusplus
}
#endif

#endif
