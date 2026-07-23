/**
 * @file    bsp_pwm.c
 * @brief   PWM 底层驱动实现 — 三相六路 PWM 输出控制
 *******************************************************************************
 * @note    TIM1 高级定时器，中心对齐模式 2，20 kHz (ARR=4249)。
 *          PWM 模式 2：CNT < CCR 时低侧导通，谷点（CNT=0）附近三相低侧
 *          公共导通，用于低侧采样电阻电流采样。
 *
 *          触发链路：CH4 OC4REF → TIM1_TRGO2 → ADC1 注入组 (doc §2)
 *          CH4 仅内部使用，不配置 GPIO。
 *
 *          谷点采样时序约束 (doc §5, 用户要求第8点)：
 *            CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 *          BspPwm_LimitCompare 将 CCR 下限限制为 CS_CCR_MIN 以满足此约束。
 *
 *          电压标幺值转 CCR 公式（PWM 模式 2，占空比反向）：
 *            CCR = PWM/2 - (Voltage / 100.0) * PWM/2
 *******************************************************************************
 */

#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "tim.h"

/**
 * @brief  CCR 比较值限幅
 * @param[in] value    待限幅的整型值
 * @param[in] pwm_max  定时器周期（ARR 值）
 * @return 限幅后的有效 CCR 值（CS_CCR_MIN ~ pwm_max）
 * @note   下限 CS_CCR_MIN 确保谷点采样窗口：
 *         CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
static uint16_t BspPwm_LimitCompare(int32_t value, uint16_t pwm_max)
{
    if (value < (int32_t)CS_CCR_MIN)
    {
        return (uint16_t)CS_CCR_MIN;
    }

    if (value > (int32_t)pwm_max)
    {
        return pwm_max;
    }

    return (uint16_t)value;
}

/**
 * @brief  启动 ADC 触发输出（CH4 谷点采样触发）
 * @note   CCR4 设为 CS_ADC_TRIGGER_CCR，计数器从 0 向上计数越过 CCR4 时，
 *         CH4 OC4REF 经 TRGO2 触发 ADC1 注入组。
 * @retval HAL_OK          CH4 启动成功
 * @retval HAL_ERROR       CH4 启动失败
 * @retval HAL_BUSY        外设忙
 */
HAL_StatusTypeDef BspPwm_StartAdcTrigger(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, CS_ADC_TRIGGER_CCR);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    return HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
}

HAL_StatusTypeDef BspPwm_StartPowerOutputs(void)
{
    uint16_t pwm_zero = (uint16_t)(BspPwm_GetPeriod() / 2u);

    /* Always enable the bridge from a zero-voltage command. */
    BspPwm_SetCompare(pwm_zero, pwm_zero, pwm_zero);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BspPwm_Start(void)
{
    HAL_StatusTypeDef status;

    status = BspPwm_StartAdcTrigger();
    if (status != HAL_OK)
    {
        return status;
    }

    status = BspPwm_StartPowerOutputs();
    if (status != HAL_OK)
    {
        BspPwm_Stop();
    }

    return status;
}

/**
 * @brief  停止所有 PWM 通道输出（逆序停止）
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
 */
uint16_t BspPwm_GetPeriod(void)
{
    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
}

/**
 * @brief  获取三相 CCR 中的最小值
 * @return min(CCR1, CCR2, CCR3)
 * @note   用于采样窗口有效性检查 (doc §5)：
 *         CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
uint16_t BspPwm_GetMinCompare(void)
{
    uint16_t ccr1 = (uint16_t)__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_1);
    uint16_t ccr2 = (uint16_t)__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2);
    uint16_t ccr3 = (uint16_t)__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_3);
    uint16_t min_ccr = ccr1;

    if (ccr2 < min_ccr)
    {
        min_ccr = ccr2;
    }
    if (ccr3 < min_ccr)
    {
        min_ccr = ccr3;
    }

    return min_ccr;
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
 * @note   转换公式（PWM 模式 2，占空比反向）：
 *         CCR = PWM/2 - (U/100) * PWM/2
 *         +100 → CCR=0（高侧全开），0 → CCR=PWM/2（零电压），
 *         -100 → CCR=PWM（高侧全关，低侧全开）
 *         CCR 下限被限制为 CS_CCR_MIN，确保谷点采样窗口
 */
void BspPwm_SetVoltageABC(float ua, float ub, float uc)
{
    uint16_t pwm_max = BspPwm_GetPeriod();
    float pwm_half = (float)pwm_max * 0.5f;

    int32_t ccr1 = (int32_t)(pwm_half - (ua / 100.0f) * pwm_half);
    int32_t ccr2 = (int32_t)(pwm_half - (ub / 100.0f) * pwm_half);
    int32_t ccr3 = (int32_t)(pwm_half - (uc / 100.0f) * pwm_half);

    BspPwm_SetCompare(BspPwm_LimitCompare(ccr1, pwm_max),
                      BspPwm_LimitCompare(ccr2, pwm_max),
                      BspPwm_LimitCompare(ccr3, pwm_max));
}
