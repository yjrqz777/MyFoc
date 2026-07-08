/**
 * @file    bsp_ws2812.c
 * @brief   WS2812 RGB LED driver using TIM2 CH1 PWM DMA on PA5.
 *
 * TIM2 is configured for an 800 kHz PWM period. DMA writes CCR1 values:
 * a short high pulse means bit 0, a long high pulse means bit 1.
 */

#include "bsp_ws2812.h"

#include "tim.h"

#define WS2812_BITS_PER_LED     24u
#define WS2812_RESET_SLOTS      64u
#define WS2812_BUFFER_LEN       ((BSP_WS2812_LED_NUM * WS2812_BITS_PER_LED) + WS2812_RESET_SLOTS)
#define WS2812_TIM_CHANNEL      TIM_CHANNEL_1
#define WS2812_TIM_HANDLE       htim2

static BspWs2812_Color_t ws2812_color[BSP_WS2812_LED_NUM];
static uint16_t ws2812_pwm_buf[WS2812_BUFFER_LEN];
static volatile uint8_t ws2812_busy = 0u;

static uint16_t BspWs2812_Code0(void)
{
    uint32_t period_ticks = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&WS2812_TIM_HANDLE) + 1u;

    return (uint16_t)((period_ticks * 28u) / 100u);
}

static uint16_t BspWs2812_Code1(void)
{
    uint32_t period_ticks = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&WS2812_TIM_HANDLE) + 1u;

    return (uint16_t)((period_ticks * 56u) / 100u);
}

static void BspWs2812_EncodeByte(uint8_t value, uint16_t *buffer, uint16_t *offset)
{
    uint8_t bit;
    uint16_t code0 = BspWs2812_Code0();
    uint16_t code1 = BspWs2812_Code1();

    for (bit = 0u; bit < 8u; bit++)
    {
        if ((value & (uint8_t)(0x80u >> bit)) != 0u)
        {
            buffer[*offset] = code1;
        }
        else
        {
            buffer[*offset] = code0;
        }
        (*offset)++;
    }
}

static void BspWs2812_BuildBuffer(void)
{
    uint16_t index;
    uint16_t offset = 0u;

    for (index = 0u; index < BSP_WS2812_LED_NUM; index++)
    {
        /* WS2812 data order is GRB, not RGB. */
        BspWs2812_EncodeByte(ws2812_color[index].green, ws2812_pwm_buf, &offset);
        BspWs2812_EncodeByte(ws2812_color[index].red, ws2812_pwm_buf, &offset);
        BspWs2812_EncodeByte(ws2812_color[index].blue, ws2812_pwm_buf, &offset);
    }

    while (offset < WS2812_BUFFER_LEN)
    {
        ws2812_pwm_buf[offset] = 0u;
        offset++;
    }
}

void BspWs2812_Init(void)
{
    BspWs2812_Clear();
    __HAL_TIM_SET_COMPARE(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, 0u);
}

HAL_StatusTypeDef BspWs2812_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    return BspWs2812_SetColorIndex(0u, red, green, blue);
}

HAL_StatusTypeDef BspWs2812_SetColorIndex(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= BSP_WS2812_LED_NUM)
    {
        return HAL_ERROR;
    }

    if (ws2812_busy != 0u)
    {
        return HAL_BUSY;
    }

    ws2812_color[index].red = red;
    ws2812_color[index].green = green;
    ws2812_color[index].blue = blue;

    return HAL_OK;
}

HAL_StatusTypeDef BspWs2812_Show(void)
{
    HAL_StatusTypeDef status;

    if (ws2812_busy != 0u)
    {
        return HAL_BUSY;
    }

    BspWs2812_BuildBuffer();
    ws2812_busy = 1u;
    __HAL_TIM_SET_COUNTER(&WS2812_TIM_HANDLE, 0u);
    __HAL_TIM_SET_COMPARE(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, 0u);

    status = HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE,
                                   WS2812_TIM_CHANNEL,
                                   (uint32_t *)ws2812_pwm_buf,
                                   WS2812_BUFFER_LEN);
    if (status != HAL_OK)
    {
        ws2812_busy = 0u;
        __HAL_TIM_SET_COMPARE(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, 0u);
    }

    return status;
}

HAL_StatusTypeDef BspWs2812_WriteColor(uint8_t red, uint8_t green, uint8_t blue)
{
    HAL_StatusTypeDef status;

    status = BspWs2812_SetColor(red, green, blue);
    if (status != HAL_OK)
    {
        return status;
    }

    return BspWs2812_Show();
}

void BspWs2812_Clear(void)
{
    uint16_t index;

    if (ws2812_busy != 0u)
    {
        return;
    }

    for (index = 0u; index < BSP_WS2812_LED_NUM; index++)
    {
        ws2812_color[index].red = 0u;
        ws2812_color[index].green = 0u;
        ws2812_color[index].blue = 0u;
    }
}

uint8_t BspWs2812_IsBusy(void)
{
    return ws2812_busy;
}

void BspWs2812_TimPulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
    {
        return;
    }

    (void)HAL_TIM_PWM_Stop_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL);
    __HAL_TIM_SET_COMPARE(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, 0u);
    ws2812_busy = 0u;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    BspWs2812_TimPulseFinishedCallback(htim);
}
