#ifndef __USER_FOC_PID_H__
#define __USER_FOC_PID_H__

#include <stdint.h>

typedef struct tUserFocPidDef
{
    int32_t s32KpQ16;
    int32_t s32KiQ16;
    int32_t s32KdQ16;
    int64_t s64IntegralQ16;
    int32_t s32PreviousError;
    int32_t s32OutputMax;
    int32_t s32OutputMin;
    int32_t s32Output;
} tUserFocPidDef;

void UserFocPidInit(tUserFocPidDef *ptPid,
                    int32_t s32KpQ16,
                    int32_t s32KiQ16,
                    int32_t s32KdQ16,
                    int32_t s32OutputMax,
                    int32_t s32OutputMin);
void UserFocPidReset(tUserFocPidDef *ptPid);
int32_t UserFocPidCalculate(tUserFocPidDef *ptPid,
                            int32_t s32Setpoint,
                            int32_t s32Feedback);

/* ---- 浮点 PI 调节器(速度环等浮点控制场景使用) ---- */
typedef struct tUserSpeedPidDef
{
    float f32Kp;             /* 比例系数 */
    float f32Ki;             /* 积分系数(按采样周期归一化前) */
    float f32Integral;       /* 积分项累加值 */
    float f32OutputMax;      /* 输出上限 */
    float f32OutputMin;      /* 输出下限 */
    float f32ErrorDeadband;  /* 误差死区，小于此值视为0并清积分 */
    float f32Output;         /* 上一次输出，方便观测 */
} tUserSpeedPidDef;

void  UserSpeedPidInit(tUserSpeedPidDef *ptPid,
                       float f32Kp, float f32Ki,
                       float f32OutputMax, float f32OutputMin,
                       float f32ErrorDeadband);
void  UserSpeedPidReset(tUserSpeedPidDef *ptPid);
float UserSpeedPidCalculate(tUserSpeedPidDef *ptPid,
                            float f32Setpoint, float f32Feedback,
                            float f32LoopHz);

#endif
