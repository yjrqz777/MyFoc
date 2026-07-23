/**
 * @file    bsp_pwm.c
 * @brief   PWM 底层驱动实现 — 三相六路 PWM 输出控制
 *******************************************************************************
 * @note    TIM1 高级定时器，中心对齐模式 2，死区由 CubeMX 配置
 *          PWM 模式 2：CNT < CCR 时低侧导通，谷点（CNT=0）附近三相低侧
 *          公共导通，用于低侧采样电阻电流采样。
 *          电压标幺值转 CCR 比较值公式（PWM 模式 2，占空比反向）：
 *          CCR = PWM/2 - (Voltage / 100.0) * PWM/2
 *******************************************************************************
 */

#include "bsp_pwm.h"
#include "tim.h"

/* ADC 低侧采样电阻电流采样触发位置（谷点采样方案）。
 *
 * PWM 模式 2 + 中心对齐模式 2：CNT < CCR 时互补低侧输出有效（低侧导通），
 * 三相低侧在 CNT=0（谷点）附近形成公共导通窗口（零矢量 000）。
 *
 * TIM1 中心对齐模式 2，CH4 比较事件仅在向上计数时产生。
 * CCR4 设为从 CNT=0 起的可配置延时值，向上计数越过 CCR4 时触发 ADC1 注入组。
 *
 * 采样时序约束：
 *   samplePoint + adcTime + margin < min(CCR_A, CCR_B, CCR_C)
 *
 * 即 ADC 采样保持必须在最小相 CCR 之前完成，确保采样期间三相低侧均导通。
 * BspPwm_LimitCompare 将 CCR 下限限制为 BSP_PWM_ADC_CCR_MIN 以满足此约束。
 *
 * 谷点低侧导通时间估算：以最小相 CCR 为例，低侧在向下计数越过 CCR 时开启，
 * 到 CNT=0 已导通 CCR 个计数，加上向上计数到 CCR4 的延时，总导通时间充足。
 */

/** @brief ADC 采样触发点（从 CNT=0 向上计数后的延时，定时器计数值）
 *         500 ticks ≈ 2.9 us @170 MHz */
#define BSP_PWM_ADC_SAMPLE_POINT_TICKS   500u

/** @brief ADC 注入组 4 通道转换时间（定时器计数值，约 1.2 us @170 MHz） */
#define BSP_PWM_ADC_CONV_TIME_TICKS      200u

/** @brief 采样安全裕量（定时器计数值，约 1.2 us @170 MHz） */
#define BSP_PWM_ADC_MARGIN_TICKS         200u

/** @brief CCR 下限：确保采样完成前三相低侧均处于导通状态 */
#define BSP_PWM_ADC_CCR_MIN  (BSP_PWM_ADC_SAMPLE_POINT_TICKS + \
                              BSP_PWM_ADC_CONV_TIME_TICKS +    \
                              BSP_PWM_ADC_MARGIN_TICKS)

/**
 * @brief  CCR 比较值限幅
 * @param[in] value    待限幅的整型值
 * @param[in] pwm_max  定时器周期（ARR 值）
 * @return 限幅后的有效 CCR 值（CCR_MIN ~ pwm_max）
 * @note   下限 BSP_PWM_ADC_CCR_MIN 确保谷点采样窗口：
 *         samplePoint + adcTime + margin < min(CCR_A, CCR_B, CCR_C)
 */
static uint16_t BspPwm_LimitCompare(int32_t value, uint16_t pwm_max)
{
    if (value < (int32_t)BSP_PWM_ADC_CCR_MIN)
    {
        return BSP_PWM_ADC_CCR_MIN;
    }

    if (value > (int32_t)pwm_max)
    {
        return pwm_max;
    }

    return (uint16_t)value;
}

/**
 * @brief  启动 ADC 触发输出（CH4 谷点采样触发）
 * @note   CCR4 设为 BSP_PWM_ADC_SAMPLE_POINT_TICKS，计数器从 0 向上
 *         计数越过 CCR4 时触发 ADC1 注入组
 * @retval HAL_OK          CH4 启动成功
 * @retval HAL_ERROR       CH4 启动失败
 * @retval HAL_BUSY        外设忙
 * @see    BspPwm_Stop
 */
HAL_StatusTypeDef BspPwm_StartAdcTrigger(void)
{
    uint16_t trigger = BSP_PWM_ADC_SAMPLE_POINT_TICKS;

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, trigger);
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
 * @note   转换公式（PWM 模式 2，占空比反向）：
 *         CCR = PWM/2 - (U/100) * PWM/2
 *         +100 → CCR=0（高侧全开，最大正电压），0 → CCR=PWM/2（零电压），
 *         -100 → CCR=PWM（高侧全关，最大负电压）
 *         CCR 下限被限制为 BSP_PWM_ADC_CCR_MIN，确保谷点采样窗口
 *         输出值自动限幅至 [CCR_MIN, PWM_PERIOD]
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
