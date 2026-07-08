/**
 * @file    bsp_ws2812.h
 * @brief   WS2812 RGB LED driver using TIM2 CH1 PWM DMA on PA5.
 */

#ifndef __BSP_WS2812_H__
#define __BSP_WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BSP_WS2812_LED_NUM 1u

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} BspWs2812_Color_t;

void BspWs2812_Init(void);
HAL_StatusTypeDef BspWs2812_SetColor(uint8_t red, uint8_t green, uint8_t blue);
HAL_StatusTypeDef BspWs2812_SetColorIndex(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
HAL_StatusTypeDef BspWs2812_Show(void);
HAL_StatusTypeDef BspWs2812_WriteColor(uint8_t red, uint8_t green, uint8_t blue);
void BspWs2812_Clear(void);
uint8_t BspWs2812_IsBusy(void);
void BspWs2812_TimPulseFinishedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WS2812_H__ */
