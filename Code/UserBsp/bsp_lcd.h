/**
 * @file    bsp_lcd.h
 * @brief   LCD 显示底层驱动头文件
 *******************************************************************************
 * @note    封装 ST7789V 驱动，提供应用层便捷显示接口
 *******************************************************************************
 */

#ifndef __BSP_LCD_H__
#define __BSP_LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BspLcd_Init(void);
void BspLcd_ShowUInt(uint16_t x, uint16_t y, uint32_t value, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LCD_H__ */
