#ifndef __RGB_H
#define __RGB_H
#include "main.h"


#define WSH 140
#define WSL 70

#define PIXEL_NUM  15
#define NUM (24*PIXEL_NUM + 350)        // Reset 280us / 1.25us = 224

 
 
void WS281x_SetAll(uint32_t GRBColor);
void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor);
void WS_CloseAll(void);
void WS_Load(void);
uint32_t rgb2grb(uint32_t rgb);

#endif
 
 