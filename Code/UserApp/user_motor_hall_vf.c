/**
 * @file    user_motor_hall_vf.c
 * @brief   Hall feedback checks and V/F open-loop startup support.
 */

#include "user_motor_hall_vf.h"
#include "bsp_hall.h"
#include "bsp_pwm.h"
#include <math.h>

#define USER_MOTOR_HALL_FOC_ENTRY_SPEED      (150.0f)
#define USER_MOTOR_HALL_FOC_EXIT_SPEED       (140.0f)
#define USER_MOTOR_HALL_SPEED_RATIO_MIN      (0.50f)
#define USER_MOTOR_HALL_SPEED_RATIO_MAX      (2.00f)
#define USER_MOTOR_HALL_SPEED_JUMP_MIN       (30.0f)
#define USER_MOTOR_HALL_SPEED_JUMP_RATIO     (0.50f)

#define USER_MOTOR_VF_START_VOLTAGE          (5.0f)
#define USER_MOTOR_VF_RUN_VOLTAGE            (12.0f)
#define USER_MOTOR_VF_VOLTAGE_RAMP_STEP      (0.0002f)
#define USER_MOTOR_VF_START_THETA_STEP       (0.0003f)
#define USER_MOTOR_VF_THETA_STEP_RAMP        (0.0000002f)
#define USER_MOTOR_VF_DIRECTION              (1.0f)
#define M_2PI                    (6.28318531f)

/**
 * @brief   计算浮点数绝对值
 * @param[in] f32Value  输入值
 * @return  输入值的绝对值
 */
static float UserMotorHallVfAbsFloat(float f32Value)
{
    return (f32Value >= 0.0f) ? f32Value : -f32Value;
}

/**
 * @brief   浮点数按固定步进斜坡逼近目标值
 * @param[in] f32Current  当前值
 * @param[in] f32Target   目标值
 * @param[in] f32Step     单次逼近步进(正值)
 * @return  斜坡后的当前值，越过目标时钳位到目标值
 */
static float UserMotorHallVfRampFloat(float f32Current,
                                      float f32Target,
                                      float f32Step)
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
 * @brief   初始化 Hall 开环起动状态
 * @param[in,out] ptState  状态结构体指针，为 NULL 时直接返回
 * @note    开环角度置 0、角度步进取起始步进、电压置 0，并清除 Hall 校验历史与 FOC 锁定计数
 */
void UserMotorHallVfInit(tUserMotorHallVfStateDef *ptState)
{
    if (ptState == NULL)
    {
        return;
    }

    ptState->f32OpenLoopTheta = 0.0f;
    ptState->f32OpenLoopStep = USER_MOTOR_VF_START_THETA_STEP;
    ptState->f32OpenVoltage = 0.0f;
    ptState->f32HallCheckLastSpeed = 0.0f;
    ptState->u32HallFocLockCounter = 0u;
}

/**
 * @brief   按给定角度与幅值合成电压矢量并输出到三相 PWM
 * @param[in] f32Theta   电压矢量角度(rad)
 * @param[in] f32Voltage 电压矢量幅值(V)
 * @note    由角度正弦/余弦得 α/β 电压，经逆 Clarke 变换得三相电压，调用
 *          BspPwmSetVoltageAbc 输出
 */
void UserMotorHallVfSetVoltageVector(float f32Theta, float f32Voltage)
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

/**
 * @brief   执行 V/F 开环起动：斜坡更新角度步进与电压并输出
 * @param[in,out] ptState              状态结构体指针
 * @param[in]     f32SpeedRef          目标速度(rad/s)
 * @param[in]     u32FastLoopHz        快速环频率(Hz)，用于将速度换算为每拍角度增量
 * @param[in]     f32SpeedReferenceMax 速度参考上限(rad/s)
 * @note    参数非法(空指针/频率为 0/上限≤0)时直接返回；角度增量按速度比斜坡变化，
 *          电压在起始电压与运行电压之间按速度比线性插值，输出角度规整到 [0, 2π)
 */
void UserMotorHallVfRunStartupOpenLoop(tUserMotorHallVfStateDef *ptState,
                                       float f32SpeedRef,
                                       uint32_t u32FastLoopHz,
                                       float f32SpeedReferenceMax)
{
    float Ratio;
    float TargetStep;
    float TargetVoltage;

    if ((ptState == NULL) || (u32FastLoopHz == 0u) ||
        (f32SpeedReferenceMax <= 0.0f))
    {
        return;
    }

    Ratio = f32SpeedRef / f32SpeedReferenceMax;
    if (Ratio < 0.0f)
    {
        Ratio = 0.0f;
    }
    else if (Ratio > 1.0f)
    {
        Ratio = 1.0f;
    }

    TargetStep = f32SpeedRef / (float)u32FastLoopHz;
    if (TargetStep > (f32SpeedReferenceMax / (float)u32FastLoopHz))
    {
        TargetStep = f32SpeedReferenceMax / (float)u32FastLoopHz;
    }

    TargetVoltage = USER_MOTOR_VF_START_VOLTAGE +
                    ((USER_MOTOR_VF_RUN_VOLTAGE - USER_MOTOR_VF_START_VOLTAGE) * Ratio);
    ptState->f32OpenLoopStep = UserMotorHallVfRampFloat(
        ptState->f32OpenLoopStep, TargetStep, USER_MOTOR_VF_THETA_STEP_RAMP);
    ptState->f32OpenVoltage = UserMotorHallVfRampFloat(
        ptState->f32OpenVoltage, TargetVoltage, USER_MOTOR_VF_VOLTAGE_RAMP_STEP);
    ptState->f32OpenLoopTheta += USER_MOTOR_VF_DIRECTION * ptState->f32OpenLoopStep;

    while (ptState->f32OpenLoopTheta >= M_2PI)
    {
        ptState->f32OpenLoopTheta -= M_2PI;
    }
    while (ptState->f32OpenLoopTheta < 0.0f)
    {
        ptState->f32OpenLoopTheta += M_2PI;
    }

    UserMotorHallVfSetVoltageVector(ptState->f32OpenLoopTheta,
                                    ptState->f32OpenVoltage);
}

/**
 * @brief   判断 Hall 速度是否满足切入 FOC 闭环的条件
 * @param[in,out] ptState       状态结构体指针，内部记录上次 Hall 速度用于跳变检测
 * @param[in]     f32HallSpeed  Hall 电角速度(rad/s)
 * @param[in]     u32FastLoopHz 快速环频率(Hz)
 * @retval 1  满足切入条件
 * @retval 0  不满足条件
 * @note    要求 Hall 角度有效且偏移已校准；实际速度须与开环期望速度同向且比值落在
 *          [SPEED_RATIO_MIN, SPEED_RATIO_MAX] 内，两者均达到切入速度阈值；锁定期间
 *          相邻两次速度跳变超过限值视为异常
 */
uint8_t UserMotorHallVfIsReadyForFoc(tUserMotorHallVfStateDef *ptState,
                                     float f32HallSpeed,
                                     uint32_t u32FastLoopHz)
{
    float ExpectedSpeed;
    float HallAbs;
    float ExpectedAbs;
    float SpeedJump;
    float JumpLimit;

    if ((ptState == NULL) || (u32FastLoopHz == 0u) ||
        (BspHallIsAngleValid() == 0u) ||
        (BspHallIsOffsetCalibrated() == 0u))
    {
        return 0u;
    }

    ExpectedSpeed = -USER_MOTOR_VF_DIRECTION * ptState->f32OpenLoopStep *
                    (float)u32FastLoopHz;
    HallAbs = UserMotorHallVfAbsFloat(f32HallSpeed);
    ExpectedAbs = UserMotorHallVfAbsFloat(ExpectedSpeed);
    SpeedJump = UserMotorHallVfAbsFloat(
        f32HallSpeed - ptState->f32HallCheckLastSpeed);
    JumpLimit = USER_MOTOR_HALL_SPEED_JUMP_MIN +
                (ExpectedAbs * USER_MOTOR_HALL_SPEED_JUMP_RATIO);

    ptState->f32HallCheckLastSpeed = f32HallSpeed;
    if ((ExpectedAbs < USER_MOTOR_HALL_FOC_ENTRY_SPEED) ||
        (HallAbs < USER_MOTOR_HALL_FOC_ENTRY_SPEED) ||
        ((f32HallSpeed * ExpectedSpeed) <= 0.0f) ||
        (HallAbs < (ExpectedAbs * USER_MOTOR_HALL_SPEED_RATIO_MIN)) ||
        (HallAbs > (ExpectedAbs * USER_MOTOR_HALL_SPEED_RATIO_MAX)) ||
        ((ptState->u32HallFocLockCounter != 0u) && (SpeedJump > JumpLimit)))
    {
        return 0u;
    }

    return 1u;
}

/**
 * @brief   判断是否应退出 FOC 闭环
 * @param[in] f32SpeedRef   速度参考(rad/s)
 * @param[in] f32HallSpeed  Hall 电角速度(rad/s)
 * @retval 1  应退出 FOC
 * @retval 0  继续 FOC
 * @note    Hall 无效/未校准、Hall 速度低于退出阈值，或速度参考仍高于阈值但反馈
 *          方向相反时要求退出
 */
uint8_t UserMotorHallVfShouldExitFoc(float f32SpeedRef,
                                     float f32HallSpeed)
{
    float ExpectedSpeed = -USER_MOTOR_VF_DIRECTION * f32SpeedRef;

    if ((BspHallIsAngleValid() == 0u) ||
        (BspHallIsOffsetCalibrated() == 0u) ||
        (UserMotorHallVfAbsFloat(f32HallSpeed) < USER_MOTOR_HALL_FOC_EXIT_SPEED) ||
        ((UserMotorHallVfAbsFloat(ExpectedSpeed) >= USER_MOTOR_HALL_FOC_EXIT_SPEED) &&
         ((f32HallSpeed * ExpectedSpeed) <= 0.0f)))
    {
        return 1u;
    }

    return 0u;
}
