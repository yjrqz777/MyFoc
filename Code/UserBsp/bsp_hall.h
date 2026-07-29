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

/** @brief Hall 快速环频率 (Hz)，用于角度插值 */
#define BSP_HALL_CONTROL_FREQ_HZ    (20000u)

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

/* ---- Hall 角度跟踪（插值法）---- */

/**
 * @brief  初始化 Hall 角度跟踪状态
 * @note   在电机 ALIGN 开始时调用，清零角度和速度，记录当前 Hall 状态为基准。
 */
void BspHallAngleInit(void);

/**
 * @brief  Hall 角度跟踪更新（在 20kHz 快速环中调用）
 * @note   检测 Hall 状态跳变，更新转速估计，插值输出连续电角度。
 */
void BspHallAngleUpdate(void);

/**
 * @brief  获取插值后的电角度
 * @return 电角度 (rad)，范围 [0, 2π)
 */
float BspHallGetElectricalAngle(void);

/**
 * @brief  获取电角速度
 * @return 电角速度 (rad/s)，正为正转，负为反转
 */
float BspHallGetElectricalSpeed(void);

/**
 * @brief  查询角度跟踪是否有效
 * @retval 1  已检测到至少 2 次 Hall 跳变，角度和速度可用
 * @retval 0  尚未有效
 */
uint8_t BspHallIsAngleValid(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_HALL_H__ */
