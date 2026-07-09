/**
 * @file    user_motor.c
 * @brief   电机控制应用层实现 — 开 V/F 开环启动 + FOC 电流闭环
 *******************************************************************************
 * @note    使用开环 V/F 控制方式启动电机，角度步进从 INIT 逐步递增至 MAX，
 *          实现软启动。FOC 电流环叠加在基波电压上进行闭环调节。
 *
 *         控制流程：FastLoop 中每周期执行：
 *          1. 更新开环角度（斜坡加速）
 *          2. 计算三相正弦基波电压
 *          3. 采样三相电流
 *          4. FOC 电流环调节（Clarke → Park → PI → 反 Park → 反 Clarke）
 *          5. 合成电压输出到 PWM
 *******************************************************************************
 */

#include "user_motor.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "user_foc.h"
#include <math.h>



#define USER_MOTOR_PI              3.14159265f
#define USER_MOTOR_TWO_PI          (2.0f * USER_MOTOR_PI)
#define USER_MOTOR_BASE_VOLTAGE    55.0f       /**< 基波电压幅值 */
#define USER_MOTOR_THETA_STEP_INIT 0.0015f     /**< 开环角度步进初始值 */
#define USER_MOTOR_THETA_STEP_MAX  0.006f      /**< 开环角度步进最大值 */
#define USER_MOTOR_THETA_STEP_INC  0.0000005f  /**< 步进递增增量 */

/** @brief 当前开环电角度（弧度） */
static float s_open_loop_theta USER_MOTOR_CCMRAM;

/** @brief 当前开环角度步进（决定转速） */
static float s_open_loop_step USER_MOTOR_CCMRAM;

/**
 * @brief  更新开环角度（斜坡加速）
 * @note   每调用一次，角度增加当前步进值
 *         步进从 INIT 逐渐递增至 MAX，实现软启动
 *         角度超过 2π 时回绕
 */
static void UserMotor_UpdateOpenLoopTheta(void)
{
    if (s_open_loop_step < USER_MOTOR_THETA_STEP_MAX)
    {
        s_open_loop_step += USER_MOTOR_THETA_STEP_INC;
    }

    s_open_loop_theta += s_open_loop_step;
    if (s_open_loop_theta > USER_MOTOR_TWO_PI)
    {
        s_open_loop_theta -= USER_MOTOR_TWO_PI;
    }
}

/**
 * @brief  初始化电机控制参数
 * @note   角度和步进清零，恢复到初始状态
 */
void UserMotor_Init(void)
{
    s_open_loop_theta = 0.0f;
    s_open_loop_step = USER_MOTOR_THETA_STEP_INIT;
}

/**
 * @brief  启动电机（ADC 注入采样 + PWM 输出）
 * @retval HAL_OK      启动成功
 * @retval HAL_ERROR   ADC 校准失败
 * @retval HAL_BUSY    外设忙
 * @note   必须先初始化 ADC 和 PWM，再调用此函数
 */
HAL_StatusTypeDef UserMotor_Start(void)
{
    HAL_StatusTypeDef status;

    status = BspAdc_StartInjected();
    if (status != HAL_OK)
    {
        return status;
    }

    return BspPwm_Start();
}

/**
 * @brief  电机快速控制循环（应在 10kHz 定时中断中调用）
 * @note   执行以下步骤：
 *         - 更新开环角度（斜坡加速）
 *         - 计算三相正弦基波电压（V/F 控制）
 *         - 采集三相电流
 *         - FOC 电流闭环调节（Id=0, Iq=6A 控制策略）
 *         - 基波电压 + FOC 补偿电压合成
 *         - 更新 PWM 比较寄存器
 */
USER_MOTOR_FAST_CODE void UserMotor_FastLoop(void)
{
    float ia;
    float ib;
    float ic;
    float ua_base;
    float ub_base;
    float uc_base;
    ThreePhaseVoltage_t uabc_foc;

    UserMotor_UpdateOpenLoopTheta();

    /* V/F 开环：三相正弦基波电压 */
    ua_base = USER_MOTOR_BASE_VOLTAGE * sinf(s_open_loop_theta);
    ub_base = USER_MOTOR_BASE_VOLTAGE * sinf(s_open_loop_theta - 2.0f * USER_MOTOR_PI / 3.0f);
    uc_base = USER_MOTOR_BASE_VOLTAGE * sinf(s_open_loop_theta - 4.0f * USER_MOTOR_PI / 3.0f);

    /* 采样三相电流 */
    ia = BspAdc_GetIa();
    ib = BspAdc_GetIb();
    ic = BspAdc_GetIc();

    /* FOC 电流闭环调节 */
    FOC_CurrentLoop(ia, ib, ic, s_open_loop_theta);
    uabc_foc = FOC_GetThreePhaseVoltage();

    /* 合成电压：基波 + FOC 补偿，输出到 PWM */
    BspPwm_SetVoltageABC(ua_base + uabc_foc.ua,
                         ub_base + uabc_foc.ub,
                         uc_base + uabc_foc.uc);
}

/**
 * @brief  获取当前开环电角度
 * @return 当前开环转子电角度（弧度，0~2π）
 */
float UserMotor_GetOpenLoopTheta(void)
{
    return s_open_loop_theta;
}
