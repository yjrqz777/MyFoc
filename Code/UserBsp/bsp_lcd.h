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

#include "user_global.h"

void BspLcdInit(void);
void BspLcdShowUInt(uint16_t u16X, uint16_t u16Y, uint32_t u32Value, uint8_t u8Length);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LCD_H__ */
