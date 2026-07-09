/**
 * @file    user_motor.c
 * @brief   电机控制应用层实现 — 开环角度 + FOC 电流闭环
 *******************************************************************************
 * @note    使用开环电角度运行电流环，角度步进从 INIT 逐步递增至 MAX。
 *
 *         控制流程：FastLoop 中每周期执行：
 *          1. 更新开环角度（斜坡加速）
 *          2. 采样三相电流
 *          3. FOC 电流环调节（Clarke → Park → PI → 反 Park → 反 Clarke）
 *          4. 输出闭环电压到 PWM
 *******************************************************************************
 */

#include "user_motor.h"
#include "bsp_adc.h"
#include "bsp_pwm.h"
#include "user_foc.h"
#include <math.h>



#define USER_MOTOR_PI              3.14159265f
#define USER_MOTOR_TWO_PI          (2.0f * USER_MOTOR_PI)
#define USER_MOTOR_IQ_REF_MAX      1.5f        /**< ADC2 SHA 满量程对应的最大 Iq，单位 A */
#define USER_MOTOR_IQ_REF_STEP     0.005f     /**< Iq 参考斜坡步进，20kHz 下约 1A/s */
#define USER_MOTOR_ADC_FULL_SCALE  4095.0f
#define USER_MOTOR_IQ_ADC_DEADZONE 300u
#define USER_MOTOR_THETA_STEP_INIT 0.0015f     /**< 开环角度步进初始值 */
#define USER_MOTOR_THETA_STEP_MAX  0.006f      /**< 开环角度步进最大值 */
#define USER_MOTOR_THETA_STEP_INC  0.0000005f  /**< 步进递增增量 */

/** @brief 当前开环电角度（弧度） */
static float s_open_loop_theta USER_MOTOR_CCMRAM;

/** @brief 当前开环角度步进（决定转速） */
static float s_open_loop_step USER_MOTOR_CCMRAM;
static float s_iq_ref USER_MOTOR_CCMRAM;

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
 * @brief  初始化电机控制参数
 * @note   角度和步进清零，恢复到初始状态
 */
void UserMotor_Init(void)
{
    s_open_loop_theta = 0.0f;
    s_open_loop_step = USER_MOTOR_THETA_STEP_INIT;
    s_iq_ref = 0.0f;
    FOC_Reset();
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
    status = BspPwm_Start();
    
    return status;
}

/**
 * @brief  电机快速控制循环（ADC 注入完成中断中调用，当前约 20kHz）
 * @note   执行以下步骤：
 *         - 更新开环角度（斜坡加速）
 *         - 采集三相电流
 *         - FOC 电流闭环调节（Id=0, Iq 斜坡给定）
 *         - 更新 PWM 比较寄存器
 */
USER_MOTOR_FAST_CODE void UserMotor_FastLoop(void)
{
    float ia;
    float ib;
    float ic;
    float iq_ref_target;
    ThreePhaseVoltage_t uabc_foc;

    if (BspAdc_IsCurrentOffsetReady() == 0u)
    {
        BspPwm_SetVoltageABC(0.0f, 0.0f, 0.0f);
        return;
    }

    UserMotor_UpdateOpenLoopTheta();

    /* 采样三相电流 */
    ia = BspAdc_GetIa();
    ib = BspAdc_GetIb();
    ic = BspAdc_GetIc();

    // iq_ref_target = UserMotor_GetIqRefTarget();
    iq_ref_target = 0.1;
    if (s_iq_ref < iq_ref_target)
    {
        s_iq_ref += USER_MOTOR_IQ_REF_STEP;
        if (s_iq_ref > iq_ref_target)
        {
            s_iq_ref = iq_ref_target;
        }
    }
    else if (s_iq_ref > iq_ref_target)
    {
        s_iq_ref -= USER_MOTOR_IQ_REF_STEP;
        if (s_iq_ref < iq_ref_target)
        {
            s_iq_ref = iq_ref_target;
        }
    }
    FOC_SetCurrentReference(0.0f, s_iq_ref);

    /* FOC 电流闭环调节 */
    FOC_CurrentLoop(ia, ib, ic, s_open_loop_theta);
    uabc_foc = FOC_GetThreePhaseVoltage();
BspPwm_SetVoltageABC(3.0f, -1.5f, -1.5f);
return;
    BspPwm_SetVoltageABC(uabc_foc.ua, uabc_foc.ub, uabc_foc.uc);
}

/**
 * @brief  获取当前开环电角度
 * @return 当前开环转子电角度（弧度，0~2π）
 */
float UserMotor_GetOpenLoopTheta(void)
{
    return s_open_loop_theta;
}

float UserMotor_GetIqRef(void)
{
    return s_iq_ref;
}
