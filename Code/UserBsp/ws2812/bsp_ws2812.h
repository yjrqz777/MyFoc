/**
 * @file    bsp_ws2812.h
 * @brief   WS2812 驱动：TIM2 CH1 PWM + Circular DMA + 双半区小缓冲区。
 *
 * 实现规格见 doc/new_ws2812.md：
 *  - 使用 HAL_TIM_PWM_Start_DMA() 启动，CC1 DMA 请求，Circular 模式。
 *  - 固定小缓冲区：2 半区 × 4 颗 LED × 24 bit = 192 个 CCR。
 *  - HT 回调填充前半区，TC 回调填充后半区，CPU 不逐位进中断。
 *  - 状态机：IDLE → DATA → RESET → DRAIN → IDLE。
 *  - 帧首前导低电平 + 帧尾 Reset 低电平（默认 ≥300us）。
 *  - DRAIN：等待最后 Reset 所在半区真正发送完成后再停止 DMA。
 */

#ifndef __BSP_WS2812_H__
#define __BSP_WS2812_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BSP_WS2812_LED_NUM (1u)

typedef struct tBspWs2812ColorDef
{
    uint8_t u8Red;
    uint8_t u8Green;
    uint8_t u8Blue;
} tBspWs2812ColorDef;

/**
 * @brief  初始化 WS2812 驱动
 * @note   计算 CCR 码值与 Reset Slot 数量，确保输出初始为低电平。
 *         必须在 MX_TIM2_Init() 之后调用。
 */
void BspWs2812Init(void);

/**
 * @brief  设置第 0 颗 LED 颜色（修改 RGB 显存）
 * @retval HAL_OK / HAL_BUSY / HAL_ERROR
 */
HAL_StatusTypeDef BspWs2812SetColor(uint8_t u8Red, uint8_t u8Green, uint8_t u8Blue);

/**
 * @brief  设置指定索引 LED 颜色（修改 RGB 显存）
 * @note   发送 Busy 期间禁止修改当前发送显存，将返回 HAL_BUSY。
 * @retval HAL_OK / HAL_BUSY / HAL_ERROR
 */
HAL_StatusTypeDef BspWs2812SetColorIndex(uint16_t u16Index,
                                        uint8_t u8Red,
                                        uint8_t u8Green,
                                        uint8_t u8Blue);

/**
 * @brief  异步启动一次发送
 * @note   不阻塞等待整帧结束。Busy 期间调用返回 HAL_BUSY。
 * @retval HAL_OK / HAL_BUSY / HAL_ERROR
 */
HAL_StatusTypeDef BspWs2812Show(void);

/**
 * @brief  设置第 0 颗 LED 颜色并立即启动发送
 * @retval HAL_OK / HAL_BUSY / HAL_ERROR
 */
HAL_StatusTypeDef BspWs2812WriteColor(uint8_t u8Red, uint8_t u8Green, uint8_t u8Blue);

/**
 * @brief  清空 RGB 显存（需调用 Show 后才实际输出）
 */
void BspWs2812Clear(void);

/**
 * @brief  查询驱动是否正在发送
 * @retval 1 = 忙，0 = 空闲
 */
uint8_t BspWs2812IsBusy(void);

/**
 * @brief  发送完成回调（弱定义）
 * @note   所有 LED 数据发送完毕、Reset 满足、DMA 已停止、输出保持低电平后调用。
 *         应用层可重写此函数实现自定义完成处理。
 */
void BspWs2812OnComplete(void);

/**
 * @brief  错误回调（弱定义）
 * @note   DMA 错误或状态机异常时调用，输出已停止并保持低电平。
 */
void BspWs2812OnError(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WS2812_H__ */
