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

#endif
