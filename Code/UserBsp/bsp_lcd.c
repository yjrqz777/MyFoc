/**
 * @file    bsp_lcd.c
 * @brief   LCD 显示底层驱动实现
 *******************************************************************************
 * @note    封装 ST7789V 驱动 API，简化应用层调用
 *******************************************************************************
 */

#include "bsp_lcd.h"
#include "st7789v/st7789v.h"

/**
 * @brief  初始化 LCD 显示屏
 * @note   调用 ST7789V 初始化序列，包括：
 *         - 硬件复位
 *         - 睡眠模式退出
 *         - 显示方向设置
 *         - Gamma 校正
 *         - 显示开启
 *         初始化完成后显示白色背景
 */
void BspLcd_Init(void)
{
    st7789v_init();
}

/**
 * @brief  在 LCD 指定位置显示无符号整数
 * @param[in] x      横坐标（像素）
 * @param[in] y      纵坐标（像素）
 * @param[in] value  待显示的无符号整数值
 * @param[in] len    显示位数
 * @note   使用白色字体、黑色背景、24 号字显示
 *         不显示前导零，不足 len 位时空格填充
 * @see    LCD_ShowIntNum
 */
void BspLcd_ShowUInt(uint16_t x, uint16_t y, uint32_t value, uint8_t len)
{
    LCD_ShowIntNum(x, y, value, len, BLACK, WHITE, 24);
}
