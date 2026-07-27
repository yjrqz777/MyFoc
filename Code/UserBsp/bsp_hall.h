/**
 * @file    bsp_hall.h
 * @brief   Hall 传感器底层驱动头文件
 *******************************************************************************
 * @note    读取三个 Hall 传感器的 GPIO 电平，组合成 3-bit 状态码（0~7）
 *          有效状态为 1~6（001~110），状态 0 和 7 为无效
 *******************************************************************************
 */

#ifndef __BSP_HALL_H__
#define __BSP_HALL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "user_global.h"

/** @brief Hall 传感器三引脚电平（u8A/u8B/u8C 对应 Hall A/B/C） */
typedef struct tBspHallPinsDef
{
    uint8_t u8A;
    uint8_t u8B;
    uint8_t u8C;
} tBspHallPinsDef;

tBspHallPinsDef BspHallReadPins(void);
uint8_t BspHallReadA(void);
uint8_t BspHallReadB(void);
uint8_t BspHallReadC(void);
uint8_t BspHallGetState(void);
uint8_t BspHallIsValidState(uint8_t u8State);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HALL_H__ */
