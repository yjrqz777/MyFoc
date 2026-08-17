#include "user_foc_pid.h"

#define USER_FOC_PID_Q_SHIFT (16)

/**
 * @brief   初始化 PI 调节器参数
 * @param[in,out] ptPid         PI 调节器指针
 * @param[in]     s32KpQ16      Kp 系数(Q16)
 * @param[in]     s32KiQ16      Ki 系数(Q16)
 * @param[in]     s32KdQ16      Kd 系数(Q16，未启用时传 0)
 * @param[in]     s32OutputMax  输出上限
 * @param[in]     s32OutputMin  输出下限
 * @note    写入参数后调用 UserFocPidReset 清零积分项与历史误差
 */
void UserFocPidInit(tUserFocPidDef *ptPid,
                    int32_t s32KpQ16,
                    int32_t s32KiQ16,
                    int32_t s32KdQ16,
                    int32_t s32OutputMax,
                    int32_t s32OutputMin)
{
    ptPid->s32KpQ16 = s32KpQ16;
    ptPid->s32KiQ16 = s32KiQ16;
    ptPid->s32KdQ16 = s32KdQ16;
    ptPid->s32OutputMax = s32OutputMax;
    ptPid->s32OutputMin = s32OutputMin;
    UserFocPidReset(ptPid);
}

/**
 * @brief   复位 PI 调节器内部状态
 * @param[in,out] ptPid  PI 调节器指针
 * @note    积分项、上一次误差与输出均清零
 */
void UserFocPidReset(tUserFocPidDef *ptPid)
{
    ptPid->s64IntegralQ16 = 0;
    ptPid->s32PreviousError = 0;
    ptPid->s32Output = 0;
}

/**
 * @brief   执行一次 PI(D) 运算并返回输出
 * @param[in,out] ptPid        PI 调节器指针，积分项与历史误差就地更新
 * @param[in]     s32Setpoint  给定值
 * @param[in]     s32Feedback  反馈值
 * @return  限幅后的 PI 输出(Q0)
 * @note    输出钳位到 [OutputMin, OutputMax]；仅当输出未饱和时更新积分项，防止积分饱和
 */
int32_t UserFocPidCalculate(tUserFocPidDef *ptPid,
                            int32_t s32Setpoint,
                            int32_t s32Feedback)
{
    int32_t s32Error = s32Setpoint - s32Feedback;
    int64_t s64IntegralCandidate;
    int64_t s64OutputQ16;
    int64_t s64MaxQ16 = (int64_t)ptPid->s32OutputMax << USER_FOC_PID_Q_SHIFT;
    int64_t s64MinQ16 = (int64_t)ptPid->s32OutputMin << USER_FOC_PID_Q_SHIFT;

    s64IntegralCandidate = ptPid->s64IntegralQ16 +
                           ((int64_t)ptPid->s32KiQ16 * s32Error);
    s64OutputQ16 = ((int64_t)ptPid->s32KpQ16 * s32Error) +
                   s64IntegralCandidate +
                   ((int64_t)ptPid->s32KdQ16 *
                    (s32Error - ptPid->s32PreviousError));

    if (s64OutputQ16 > s64MaxQ16)
    {
        s64OutputQ16 = s64MaxQ16;
    }
    else if (s64OutputQ16 < s64MinQ16)
    {
        s64OutputQ16 = s64MinQ16;
    }
    else
    {
        ptPid->s64IntegralQ16 = s64IntegralCandidate;
    }

    ptPid->s32PreviousError = s32Error;
    ptPid->s32Output = (int32_t)(s64OutputQ16 >> USER_FOC_PID_Q_SHIFT);
    return ptPid->s32Output;
}

/**
 * @brief   初始化浮点 PI 调节器参数
 * @param[in,out] ptPid             PI 调节器指针
 * @param[in]     f32Kp             比例系数
 * @param[in]     f32Ki             积分系数(按采样周期归一化前)
 * @param[in]     f32OutputMax      输出上限
 * @param[in]     f32OutputMin      输出下限
 * @param[in]     f32ErrorDeadband  误差死区，小于此值视为0并清积分
 * @note    写入参数后调用 UserSpeedPidReset 清零积分项与输出
 */
void UserSpeedPidInit(tUserSpeedPidDef *ptPid,
                       float f32Kp, float f32Ki,
                       float f32OutputMax, float f32OutputMin,
                       float f32ErrorDeadband)
{
    ptPid->f32Kp            = f32Kp;
    ptPid->f32Ki            = f32Ki;
    ptPid->f32OutputMax     = f32OutputMax;
    ptPid->f32OutputMin     = f32OutputMin;
    ptPid->f32ErrorDeadband = f32ErrorDeadband;
    UserSpeedPidReset(ptPid);
}

/**
 * @brief   复位浮点 PI 调节器内部状态
 * @param[in,out] ptPid  PI 调节器指针
 * @note    积分项与输出均清零
 */
void UserSpeedPidReset(tUserSpeedPidDef *ptPid)
{
    ptPid->f32Integral = 0.0f;
    ptPid->f32Output   = 0.0f;
}

/**
 * @brief   执行一次浮点 PI 运算并返回输出
 * @param[in,out] ptPid         PI 调节器指针，积分项就地更新
 * @param[in]     f32Setpoint   给定值
 * @param[in]     f32Feedback   反馈值
 * @param[in]     f32LoopHz     调用频率(Hz)，积分项除以该值做频率归一化
 * @return  限幅后的 PI 输出
 * @note    1) 误差死区内强制清零误差与积分项，抑制零位测速噪声
 *          2) 积分候选值先独立限幅，再叠加比例项判断是否饱和
 *          3) 条件积分抗饱和：未饱和、或饱和且误差正在反向时才更新积分项
 */
float UserSpeedPidCalculate(tUserSpeedPidDef *ptPid,
                            float f32Setpoint, float f32Feedback,
                            float f32LoopHz)
{
    float f32Error = f32Setpoint - f32Feedback;
    float f32AbsError = (f32Error >= 0.0f) ? f32Error : -f32Error;
    float f32Proportional;
    float f32IntegratorCandidate;
    float f32UnclampedOutput;

    /* 误差死区：抑制测速噪声导致的残余积分抖动 */
    if (f32AbsError < ptPid->f32ErrorDeadband)
    {
        f32Error = 0.0f;
        ptPid->f32Integral = 0.0f;
    }

    f32Proportional        = ptPid->f32Kp * f32Error;
    f32IntegratorCandidate = ptPid->f32Integral + (ptPid->f32Ki * f32Error / f32LoopHz);

    /* 积分项预限幅 */
    if (f32IntegratorCandidate > ptPid->f32OutputMax)
    {
        f32IntegratorCandidate = ptPid->f32OutputMax;
    }
    else if (f32IntegratorCandidate < ptPid->f32OutputMin)
    {
        f32IntegratorCandidate = ptPid->f32OutputMin;
    }

    f32UnclampedOutput = f32Proportional + f32IntegratorCandidate;

    /* 条件积分抗饱和：未饱和、或饱和但误差正在反向时才更新积分项 */
    if (((f32UnclampedOutput <= ptPid->f32OutputMax) && (f32UnclampedOutput >= ptPid->f32OutputMin)) ||
        ((f32UnclampedOutput >  ptPid->f32OutputMax) && (f32Error < 0.0f)) ||
        ((f32UnclampedOutput <  ptPid->f32OutputMin) && (f32Error > 0.0f)))
    {
        ptPid->f32Integral = f32IntegratorCandidate;
    }

    /* 最终限幅输出 */
    f32UnclampedOutput = f32Proportional + ptPid->f32Integral;
    if (f32UnclampedOutput > ptPid->f32OutputMax)
    {
        f32UnclampedOutput = ptPid->f32OutputMax;
    }
    else if (f32UnclampedOutput < ptPid->f32OutputMin)
    {
        f32UnclampedOutput = ptPid->f32OutputMin;
    }

    ptPid->f32Output = f32UnclampedOutput;
    return ptPid->f32Output;
}
