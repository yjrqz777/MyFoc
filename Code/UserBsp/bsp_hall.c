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
uint8_t BspHallReadA(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin);
}

/**
 * @brief  读取 Hall B 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHallReadB(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin);
}

/**
 * @brief  读取 Hall C 引脚电平
 * @retval 0  低电平
 * @retval 1  高电平
 */
uint8_t BspHallReadC(void)
{
    return (uint8_t)HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin);
}

/**
 * @brief  读取三个 Hall 引脚电平并打包为结构体
 * @return tBspHallPinsDef 结构体
 */
tBspHallPinsDef BspHallReadPins(void)
{
    tBspHallPinsDef Pins;

    Pins.u8A = BspHallReadA();
    Pins.u8B = BspHallReadB();
    Pins.u8C = BspHallReadC();

    return Pins;
}

/**
 * @brief  获取 Hall 状态码（组合三位 Hall 电平为 3-bit 值）
 * @return 状态码（0~7）
 * @note   bit0 = A 相, bit1 = B 相, bit2 = C 相
 *         有效扇区范围：1~6
 */
uint8_t BspHallGetState(void)
{
    tBspHallPinsDef Pins = BspHallReadPins();

    return (uint8_t)(Pins.u8A | (Pins.u8B << 1) | (Pins.u8C << 2));
}

/**
 * @brief  判断 Hall 状态是否有效
 * @param[in] u8State  Hall 状态码（0~7）
 * @retval 1  有效（状态在 1~6 之间）
 * @retval 0  无效（状态为 0 或 7，对应全部低或全部高）
 * @note   正常情况下 Hall 传感器不会输出 000 或 111
 */
uint8_t BspHallIsValidState(uint8_t u8State)
{
    u8State &= 0x07u;
    return (u8State != 0x00u) && (u8State != 0x07u);
}
