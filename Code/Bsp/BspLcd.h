#ifndef __BSP_LCD_H__
#define __BSP_LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void BspLcd_Init(void);
void BspLcd_ShowMotorDebug(uint8_t hall_state, const uint16_t adc_raw[4]);

#ifdef __cplusplus
}
#endif

#endif
