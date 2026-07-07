#include "bsp_pwm.h"
#include "tim.h"

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

HAL_StatusTypeDef BspPwm_Start(void)
{
    HAL_StatusTypeDef status;

    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    if (status != HAL_OK) return status;
    status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    if (status != HAL_OK) return status;

    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    if (status != HAL_OK) return status;
    status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    if (status != HAL_OK) return status;

    status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    if (status != HAL_OK) return status;
    status = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    if (status != HAL_OK) return status;

    return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

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

uint16_t BspPwm_GetPeriod(void)
{
    return (uint16_t)__HAL_TIM_GET_AUTORELOAD(&htim1);
}

void BspPwm_SetCompare(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, ccr3);
}

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
