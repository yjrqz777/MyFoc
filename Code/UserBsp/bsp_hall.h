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

#include "main.h"

/** @brief Hall 传感器三引脚电平（a/b/c 对应 Hall A/B/C） */
typedef struct {
    uint8_t a;
    uint8_t b;
    uint8_t c;
} BspHallPins_t;

BspHallPins_t BspHall_ReadPins(void);
uint8_t BspHall_ReadA(void);
uint8_t BspHall_ReadB(void);
uint8_t BspHall_ReadC(void);
uint8_t BspHall_GetState(void);
uint8_t BspHall_IsValidState(uint8_t state);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HALL_H__ */
