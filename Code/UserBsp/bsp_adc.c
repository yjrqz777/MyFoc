#include "bsp_adc.h"
#include "adc.h"

static volatile uint16_t s_injected_raw[BSP_ADC_INJECTED_CHANNELS] = {0};

static float BspAdc_RawToCurrent(uint16_t raw)
{
    const float adc_scale = 3.3f / 4096.0f / 20.0f / 0.1f;
    return ((float)raw - 2048.0f) * adc_scale;
}

HAL_StatusTypeDef BspAdc_StartInjected(void)
{
    HAL_StatusTypeDef status;

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_ADCEx_InjectedStart_IT(&hadc1);
}

uint8_t BspAdc_UpdateInjected(ADC_HandleTypeDef *hadc)
{
    if ((hadc == NULL) || (hadc->Instance != ADC1))
    {
        return 0u;
    }

    s_injected_raw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    s_injected_raw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
    s_injected_raw[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);
    s_injected_raw[3] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_4);

    return 1u;
}

uint16_t BspAdc_GetInjectedRaw(uint8_t index)
{
    if (index >= BSP_ADC_INJECTED_CHANNELS)
    {
        return 0u;
    }

    return s_injected_raw[index];
}

float BspAdc_GetIa(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[0]);
}

float BspAdc_GetIb(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[1]);
}

float BspAdc_GetIc(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[2]);
}

float BspAdc_GetIbus(void)
{
    return BspAdc_RawToCurrent(s_injected_raw[3]);
}
