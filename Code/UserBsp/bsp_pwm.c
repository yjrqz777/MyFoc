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
 *          BspPwmLimitCompare 将 CCR 下限限制为 CS_CCR_MIN 以满足此约束。
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
 * @param[in] s32Value    待限幅的整型值
 * @param[in] u16PwmMax  定时器周期（ARR 值）
 * @return 限幅后的有效 CCR 值（CS_CCR_MIN ~ u16PwmMax）
 * @note   下限 CS_CCR_MIN 确保谷点采样窗口：
 *         CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
static uint16_t BspPwmLimitCompare(int32_t s32Value, uint16_t u16PwmMax)
{
    if (s32Value < (int32_t)CS_CCR_MIN)
    {
        return (uint16_t)CS_CCR_MIN;
    }

    if (s32Value > (int32_t)u16PwmMax)
    {
        return u16PwmMax;
    }

    return (uint16_t)s32Value;
}

/**
 * @brief  启动 ADC 触发输出（CH4 谷点采样触发）
 * @note   CCR4 设为 CS_ADC_TRIGGER_CCR，计数器从 0 向上计数越过 CCR4 时，
 *         CH4 OC4REF 经 TRGO2 触发 ADC1 注入组。
 * @retval HAL_OK          CH4 启动成功
 * @retval HAL_ERROR       CH4 启动失败
 * @retval HAL_BUSY        外设忙
 */
HAL_StatusTypeDef BspPwmStartAdcTrigger(void)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, CS_ADC_TRIGGER_CCR);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0u, 0u);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    return HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
}

HAL_StatusTypeDef BspPwmStartPowerOutputs(void)
{
    uint16_t PwmZero = (uint16_t)(BspPwmGetPeriod() / 2u);

    /* Always enable the bridge from a zero-voltage command. */
    BspPwmSetCompare(PwmZero, PwmZero, PwmZero);

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

HAL_StatusTypeDef BspPwmStart(void)
{
    HAL_StatusTypeDef Status;

    Status = BspPwmStartAdcTrigger();
    if (Status != HAL_OK)
    {
        return Status;
    }

    Status = BspPwmStartPowerOutputs();
    if (Status != HAL_OK)
    {
        BspPwmStop();
    }

    return Status;
}

/**
 * @brief  停止所有 PWM 通道输出（逆序停止）
 */
void BspPwmStop(void)
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
uint16_t BspPwmGetPeriod(void)
{
    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
}

/**
 * @brief  获取三相 CCR 中的最小值
 * @return min(CCR1, CCR2, CCR3)
 * @note   用于采样窗口有效性检查 (doc §5)：
 *         CCR4 + ADC_SAMPLE_TICKS + BLANK_TICKS <= min(CCR1, CCR2, CCR3)
 */
uint16_t BspPwmGetMinCompare(void)
{
    uint16_t Ccr1 = (uint16_t)__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_1);
    uint16_t Ccr2 = (uint16_t)__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_2);
    uint16_t Ccr3 = (uint16_t)__HAL_TIM_GetCompare(&htim1, TIM_CHANNEL_3);
    uint16_t MinCcr = Ccr1;

    if (Ccr2 < MinCcr)
    {
        MinCcr = Ccr2;
    }
    if (Ccr3 < MinCcr)
    {
        MinCcr = Ccr3;
    }

    return MinCcr;
}

/**
 * @brief  直接设置三相 PWM 比较寄存器
 * @param[in] u16Ccr1  CH1 比较值（A 相上管）
 * @param[in] u16Ccr2  CH2 比较值（B 相上管）
 * @param[in] u16Ccr3  CH3 比较值（C 相上管）
 */
void BspPwmSetCompare(uint16_t u16Ccr1, uint16_t u16Ccr2, uint16_t u16Ccr3)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, u16Ccr1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, u16Ccr2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, u16Ccr3);
}

/**
 * @brief  以电压标幺值设置三相 PWM 占空比
 * @param[in] f32Ua  A 相电压标幺值（-100 ~ 100）
 * @param[in] f32Ub  B 相电压标幺值（-100 ~ 100）
 * @param[in] f32Uc  C 相电压标幺值（-100 ~ 100）
 * @note   转换公式（PWM 模式 2，占空比反向）：
 *         CCR = PWM/2 - (U/100) * PWM/2
 *         +100 → CCR=0（高侧全开），0 → CCR=PWM/2（零电压），
 *         -100 → CCR=PWM（高侧全关，低侧全开）
 *         CCR 下限被限制为 CS_CCR_MIN，确保谷点采样窗口
 */
void BspPwmSetVoltageAbc(float f32Ua, float f32Ub, float f32Uc)
{
    uint16_t PwmMax = BspPwmGetPeriod();
    float PwmHalf = (float)PwmMax * 0.5f;

    int32_t Ccr1 = (int32_t)(PwmHalf - (f32Ua / 100.0f) * PwmHalf);
    int32_t Ccr2 = (int32_t)(PwmHalf - (f32Ub / 100.0f) * PwmHalf);
    int32_t Ccr3 = (int32_t)(PwmHalf - (f32Uc / 100.0f) * PwmHalf);

    BspPwmSetCompare(BspPwmLimitCompare(Ccr1, PwmMax),
                      BspPwmLimitCompare(Ccr2, PwmMax),
                      BspPwmLimitCompare(Ccr3, PwmMax));
}
