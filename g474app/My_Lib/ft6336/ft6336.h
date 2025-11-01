#ifndef FT_6336_H
#define FT_6336_H

#include "stm32f4xx_hal.h"


#define FT6636ADDR 0X38


extern uint8_t do_i2cdetect_cmd();
extern void ft6336_read_xy(uint16_t *x,uint16_t *y);
extern uint8_t ft6336_enable();

extern uint16_t ft6336_read_x();
extern uint16_t ft6336_read_y();

#endif // FT_6336_H

