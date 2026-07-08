/**
 * @file    bsp_hall.c
 * @brief   Hall 传感器底层驱动实现
 *******************************************************************************
 * @note    通过 GPIO 读取 Hall 传感器三路电平，组合为 3-bit 状态码
 *          用于电机转子位置检测（六个有效扇区）
 *******************************************************************************
 */

#include "bsp_hall.h"

/**
 * @brief  读取 Hall A 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHall_ReadA(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
}

/**
 * @brief  读取 Hall B 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHall_ReadB(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
}

/**
 * @brief  读取 Hall C 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHall_ReadC(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);
}

/**
 * @brief  读取三个 Hall 引脚电平并打包为结构体
 * @return BspHallPins_t 结构体
 */
BspHallPins_t BspHall_ReadPins(void)
{
    BspHallPins_t pins;

    pins.a = BspHall_ReadA();
    pins.b = BspHall_ReadB();
    pins.c = BspHall_ReadC();

    return pins;
}

/**
 * @brief  获取 Hall 状态码（组合三位 Hall 电平为 3-bit 值）
 * @return 状态码（0~7）
 * @note   bit0 = a 相, bit1 = b 相, bit2 = c 相
 *         有效扇区范围：1~6
 */
uint8_t BspHall_GetState(void)
{
    BspHallPins_t pins = BspHall_ReadPins();
    return (uint8_t)(pins.a | (pins.b << 1) | (pins.c << 2));
}

/**
 * @brief  判断 Hall 状态是否有效
 * @param[in] state  Hall 状态码（0~7）
 * @retval 1  有效（状态在 1~6 之间）
 * @retval 0  无效（状态为 0 或 7，对应全部低或全部高）
 * @note   正常情况下 Hall 传感器不会输出 000 或 111
 */
uint8_t BspHall_IsValidState(uint8_t state)
{
    state &= 0x07u;
    return (state != 0x00u) && (state != 0x07u);
}
