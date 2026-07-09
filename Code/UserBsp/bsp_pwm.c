/**
 * @file    bsp_pwm.c
 * @brief   PWM 底层驱动实现 — 三相六路 PWM 输出控制
 *******************************************************************************
 * @note    TIM1 高级定时器，中心对齐模式，死区由 CubeMX 配置
 *          电压标幺值转 CCP 比较值公式：
 *          CCR = PWM/2 + (Voltage / 100.0) * PWM/2
 *******************************************************************************
 */

#include "bsp_pwm.h"
#include "tim.h"

/**
 * @brief  CCR 比较值限幅
 * @param[in] value    待限幅的整型值
 * @param[in] pwm_max  定时器周期（ARR 值）
 * @return 限幅后的有效 CCR 值（0 ~ pwm_max）
 */
static uint16_t BspPwm_LimitCompare(int32_t value, uint16_t pwm_max)
{
    if (value < 0)
    {
        return 0u;
    }

    if (value > (int32_t)pwm_max)
    {
        return pwm_max;
    }

    return (uint16_t)value;
}

/**
 * @brief  启动所有 PWM 通道输出
 * @note   每个通道先启主通道再启互补通道，最后启 CH4
 * @retval HAL_OK          所有通道启动成功
 * @retval HAL_ERROR       某通道启动失败（立即返回）
 * @retval HAL_BUSY        外设忙
 * @see    BspPwm_Stop
 */
HAL_StatusTypeDef BspPwm_Start(void)
{
    uint16_t pwm_zero = (uint16_t)(BspPwm_GetPeriod() / 2u);

    BspPwm_SetCompare(pwm_zero, pwm_zero, pwm_zero);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)    != HAL_OK) return HAL_ERROR;
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) return HAL_ERROR;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2)    != HAL_OK) return HAL_ERROR;
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) return HAL_ERROR;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3)    != HAL_OK) return HAL_ERROR;
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) return HAL_ERROR;

    return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief  停止所有 PWM 通道输出（逆序停止）
 * @note   先停 CH4，再逆序停 CH3N→CH3→CH2N→CH2→CH1N→CH1
 *         返回值强制忽略以避免停止中途失败导致状态不一致
 * @see    BspPwm_Start
 */
void BspPwm_Stop(void)
{
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

/**
 * @brief  获取 PWM 定时器周期值
 * @return 自动重装载寄存器（ARR）的值
 * @note   此值决定 PWM 频率：f = TIM_CLK / (ARR + 1)
 */
uint16_t BspPwm_GetPeriod(void)
{
    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
}

/**
 * @brief  直接设置三相 PWM 比较寄存器
 * @param[in] ccr1  CH1 比较值（A 相上管）
 * @param[in] ccr2  CH2 比较值（B 相上管）
 * @param[in] ccr3  CH3 比较值（C 相上管）
 */
void BspPwm_SetCompare(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, ccr3);
}

/**
 * @brief  以电压标幺值设置三相 PWM 占空比
 * @param[in] ua  A 相电压标幺值（-100 ~ 100）
 * @param[in] ub  B 相电压标幺值（-100 ~ 100）
 * @param[in] uc  C 相电压标幺值（-100 ~ 100）
 * @note   转换公式：CCR = PWM/2 + (U/100) * PWM/2
 *         -100 → 0%（最大负电压），0 → 50%（零电压），+100 → 100%（最大正电压）
 *         输出值自动限幅至 [0, PWM_PERIOD]
 */
void BspPwm_SetVoltageABC(float ua, float ub, float uc)
{
    uint16_t pwm_max = BspPwm_GetPeriod();
    float pwm_half = (float)pwm_max * 0.5f;

    int32_t ccr1 = (int32_t)(pwm_half + (ua / 100.0f) * pwm_half);
    int32_t ccr2 = (int32_t)(pwm_half + (ub / 100.0f) * pwm_half);
    int32_t ccr3 = (int32_t)(pwm_half + (uc / 100.0f) * pwm_half);

    BspPwm_SetCompare(BspPwm_LimitCompare(ccr1, pwm_max),
                      BspPwm_LimitCompare(ccr2, pwm_max),
                      BspPwm_LimitCompare(ccr3, pwm_max));
}
