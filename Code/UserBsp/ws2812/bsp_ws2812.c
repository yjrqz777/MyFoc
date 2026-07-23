/**
 * @file    bsp_ws2812.c
 * @brief   WS2812 驱动：TIM2 CH1 PWM + Circular DMA + 双半区小缓冲区。
 *
 * 实现规格见 doc/new_ws2812.md。
 *
 * 架构：
 *   TIM PWM(800kHz) → CC1 DMA 请求 → Circular DMA → 双半区小缓冲区
 *   → HT 回调填充前半区 / TC 回调填充后半区 → DRAIN 后停止 DMA
 *
 * CPU 只在 DMA 半传输/传输完成时进中断（约每 120us 一次），不逐位进中断。
 * DMA 缓冲区固定 192 个 CCR，不随灯珠数量增大。
 */

#include "bsp_ws2812.h"
#include "tim.h"

/*---------------------- 配置宏 ----------------------*/
#define WS2812_TIM_HANDLE           htim2
#define WS2812_TIM_CHANNEL          TIM_CHANNEL_1
#define WS2812_TIM_DMA_ID           TIM_DMA_ID_CC1
#define WS2812_TIM_DMA_ENABLE_BIT   TIM_DMA_CC1

/* 每颗 LED 24 bit (GRB) */
#define WS2812_BITS_PER_LED         24u
/* 每半区容纳 LED 数量，spec §9 推荐 4 */
#define WS2812_LEDS_PER_HALF        4u
#define WS2812_HALF_BUFFER_SLOTS    (WS2812_LEDS_PER_HALF * WS2812_BITS_PER_LED)   /* 96  */
#define WS2812_FULL_BUFFER_SLOTS    (WS2812_HALF_BUFFER_SLOTS * 2u)                /* 192 */

/* 帧首前导低电平 Slot 数量，spec §21 建议 2~8，用于稳定 DMA/Preload 流水线 */
#define WS2812_PREAMBLE_SLOTS       4u

/* 帧尾 Reset 时间，spec §15 默认 300us，可配置 */
#define WS2812_RESET_TIME_US        300u

/* TIM2 计数时钟：G474 下 APB 定时器时钟 = 170MHz (PSC=0) */
#define WS2812_TIM_COUNTER_CLK_HZ   170000000U

/* CCR 占空比比例，spec §3：Code0 ≈ 1/3，Code1 ≈ 2/3。需用示波器校准。 */
#define WS2812_CODE0_RATIO_NUM      1u
#define WS2812_CODE0_RATIO_DEN      3u
#define WS2812_CODE1_RATIO_NUM      2u
#define WS2812_CODE1_RATIO_DEN      3u

/* TIM2 CH1 对应 ChannelState 数组下标 */
#define WS2812_CHANNEL_STATE_INDEX  0u

/*---------------------- 状态机 ----------------------*/
typedef enum
{
    WS2812_STATE_IDLE = 0u,
    WS2812_STATE_DATA,         /* 将 RGB 编码为 CCR */
    WS2812_STATE_RESET,        /* 输出帧尾 Reset 低电平 */
    WS2812_STATE_DRAIN,        /* 等待最后半区发送完成后再停止 */
    WS2812_STATE_STOPPING,
    WS2812_STATE_ERROR,
} Ws2812State_t;

/* 半区编号：HT 回调可安全修改前半区，TC 回调可安全修改后半区 */
typedef enum
{
    WS2812_HALF_FIRST = 0u,    /* 前半区 */
    WS2812_HALF_SECOND = 1u,   /* 后半区 */
} Ws2812Half_t;

/*---------------------- 运行变量 ----------------------*/
static BspWs2812_Color_t ws2812_color[BSP_WS2812_LED_NUM];
static uint32_t ws2812_dma_buf[WS2812_FULL_BUFFER_SLOTS];

static volatile Ws2812State_t ws2812_state = WS2812_STATE_IDLE;
static volatile uint8_t ws2812_busy = 0u;

static uint16_t ws2812_led_index;        /* 当前编码的 LED 索引 */
static uint8_t  ws2812_bit_index;        /* 当前 LED 内 bit 位 0..23 */
static uint16_t ws2812_preamble_remain;  /* 帧首前导低电平剩余 Slot */
static uint16_t ws2812_reset_remain;     /* 帧尾 Reset 剩余 Slot */
static uint16_t ws2812_reset_total;      /* 帧尾 Reset 总 Slot 数 */
static Ws2812Half_t ws2812_final_half;   /* 最后 Reset 数据所在半区 */

static uint32_t ws2812_ccr_code0;
static uint32_t ws2812_ccr_code1;

/*---------------------- 应用回调（弱定义） ----------------------*/
__weak void BspWs2812_OnComplete(void)
{
}

__weak void BspWs2812_OnError(void)
{
}

/*---------------------- 底层停止 ----------------------*/
/*
 * 直接寄存器操作停止 TIM+DMA，可在 ISR 中安全调用。
 * 不依赖 HAL_TIM_PWM_Stop_DMA（其在 ISR 中调用 HAL_DMA_Abort_IT 会遗留
 * ABORT 状态），手动复位 HAL 状态以保证下次 HAL_TIM_PWM_Start_DMA 可用。
 */
static void Ws2812_HwStop(void)
{
    DMA_HandleTypeDef *hdma;

    /* 关闭 TIM DMA 请求、PWM 通道输出、定时器 */
    __HAL_TIM_DISABLE_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_DMA_ENABLE_BIT);
    WS2812_TIM_HANDLE.Instance->CCER &= ~TIM_CCER_CC1E;
    __HAL_TIM_DISABLE(&WS2812_TIM_HANDLE);

    /* 关闭并复位 DMA 通道（DMA1_Channel2） */
    hdma = WS2812_TIM_HANDLE.hdma[WS2812_TIM_DMA_ID];
    if (hdma != NULL)
    {
        __HAL_DMA_DISABLE(hdma);
        hdma->State = HAL_DMA_STATE_READY;
    }
    /* 清除 Channel2 全部 DMA 标志，避免残留中断 */
    DMA1->IFCR = DMA_IFCR_CGIF2;

    /* 输出保持低电平，计数器归零 */
    __HAL_TIM_SET_COMPARE(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL, 0u);
    __HAL_TIM_SET_COUNTER(&WS2812_TIM_HANDLE, 0u);

    /* 复位 HAL TIM 状态，保证下次 Start_DMA 不返回 HAL_BUSY/ERROR */
    WS2812_TIM_HANDLE.State = HAL_TIM_STATE_READY;
    WS2812_TIM_HANDLE.ChannelState[WS2812_CHANNEL_STATE_INDEX] = HAL_TIM_CHANNEL_STATE_READY;
}

/*---------------------- 半区填充 ----------------------*/
/*
 * 根据当前状态游程，向指定半区写入 CCR 数据。
 * DATA:   先输出前导低电平，再将 RGB 按 GRB/MSB First 编码为 CCR。
 * RESET:  输出 CCR=0 帧尾低电平，消耗 Reset Slot。
 * DRAIN:  最后 Reset 已写入，剩余位置继续填 0。
 *
 * 半区边界可能落在某颗 LED 中间，通过 led_index/bit_index 游程断点续编。
 */
static void Ws2812_FillHalf(Ws2812Half_t half)
{
    uint16_t offset = (half == WS2812_HALF_FIRST) ? 0u : WS2812_HALF_BUFFER_SLOTS;
    uint16_t remain = WS2812_HALF_BUFFER_SLOTS;

    while (remain > 0u)
    {
        switch (ws2812_state)
        {
            case WS2812_STATE_DATA:
                if (ws2812_preamble_remain > 0u)
                {
                    /* 帧首前导低电平，稳定 DMA/Preload 流水线，spec §21 */
                    ws2812_dma_buf[offset] = 0u;
                    ws2812_preamble_remain--;
                }
                else if (ws2812_led_index < BSP_WS2812_LED_NUM)
                {
                    /* 编码一位：GRB 顺序，MSB First */
                    uint8_t byte_sel = (uint8_t)(ws2812_bit_index / 8u);  /* 0=G, 1=R, 2=B */
                    uint8_t bit_in_byte = (uint8_t)(ws2812_bit_index % 8u);
                    uint8_t color_byte;
                    uint8_t mask;

                    if (byte_sel == 0u)
                    {
                        color_byte = ws2812_color[ws2812_led_index].green;
                    }
                    else if (byte_sel == 1u)
                    {
                        color_byte = ws2812_color[ws2812_led_index].red;
                    }
                    else
                    {
                        color_byte = ws2812_color[ws2812_led_index].blue;
                    }
                    mask = (uint8_t)(0x80u >> bit_in_byte);
                    ws2812_dma_buf[offset] =
                        ((color_byte & mask) != 0u) ? ws2812_ccr_code1 : ws2812_ccr_code0;

                    ws2812_bit_index++;
                    if (ws2812_bit_index >= WS2812_BITS_PER_LED)
                    {
                        ws2812_bit_index = 0u;
                        ws2812_led_index++;
                    }
                }
                else
                {
                    /* 所有 LED 数据编码完成，切换到 RESET，不消耗 Slot */
                    ws2812_state = WS2812_STATE_RESET;
                    continue;
                }
                break;

            case WS2812_STATE_RESET:
                if (ws2812_reset_remain > 0u)
                {
                    ws2812_dma_buf[offset] = 0u;
                    ws2812_reset_remain--;
                }
                else
                {
                    /* 最后 Reset 已写入，记录所在半区并进入 DRAIN，不消耗 Slot */
                    ws2812_final_half = half;
                    ws2812_state = WS2812_STATE_DRAIN;
                    continue;
                }
                break;

            case WS2812_STATE_DRAIN:
            case WS2812_STATE_STOPPING:
            case WS2812_STATE_ERROR:
            default:
                /* 继续填充低电平，保证输出安全 */
                ws2812_dma_buf[offset] = 0u;
                break;
        }

        offset++;
        remain--;
    }
}

/*---------------------- 完成与错误处理 ----------------------*/
static void Ws2812_CompleteStop(void)
{
    Ws2812_HwStop();
    ws2812_state = WS2812_STATE_IDLE;
    ws2812_busy = 0u;
    BspWs2812_OnComplete();
}

static void Ws2812_GotoError(void)
{
    Ws2812_HwStop();
    ws2812_state = WS2812_STATE_ERROR;
    ws2812_busy = 0u;
    BspWs2812_OnError();
}

/*---------------------- HAL PWM DMA 回调 ----------------------*/
/*
 * 半传输完成：前半区刚发送完，DMA 正在发送后半区。
 * 可安全修改前半区。若最后 Reset 所在半区为前半区且已发送完毕则停止。
 */
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &WS2812_TIM_HANDLE)
    {
        return;
    }
    if (ws2812_busy == 0u)
    {
        return;
    }

    if ((ws2812_state == WS2812_STATE_DRAIN) &&
        (ws2812_final_half == WS2812_HALF_FIRST))
    {
        Ws2812_CompleteStop();
    }
    else
    {
        Ws2812_FillHalf(WS2812_HALF_FIRST);
    }
}

/*
 * 传输完成（Circular）：后半区刚发送完，DMA 回到前半区。
 * 可安全修改后半区。若最后 Reset 所在半区为后半区且已发送完毕则停止。
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &WS2812_TIM_HANDLE)
    {
        return;
    }
    if (ws2812_busy == 0u)
    {
        return;
    }

    if ((ws2812_state == WS2812_STATE_DRAIN) &&
        (ws2812_final_half == WS2812_HALF_SECOND))
    {
        Ws2812_CompleteStop();
    }
    else
    {
        Ws2812_FillHalf(WS2812_HALF_SECOND);
    }
}

/* DMA 错误回调 */
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &WS2812_TIM_HANDLE)
    {
        return;
    }
    Ws2812_GotoError();
}

/*---------------------- 公开接口 ----------------------*/
void BspWs2812_Init(void)
{
    uint32_t period_ticks = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&WS2812_TIM_HANDLE) + 1u;

    /* 计算码值，spec §3 */
    ws2812_ccr_code0 = (period_ticks * WS2812_CODE0_RATIO_NUM) / WS2812_CODE0_RATIO_DEN;
    ws2812_ccr_code1 = (period_ticks * WS2812_CODE1_RATIO_NUM) / WS2812_CODE1_RATIO_DEN;

    /* Reset Slot 数量 = ceil(RESET_TIME_US * counter_clk / period_ticks) */
    ws2812_reset_total = (uint16_t)(((uint32_t)WS2812_RESET_TIME_US *
                                     (WS2812_TIM_COUNTER_CLK_HZ / 1000000u) +
                                     period_ticks - 1u) / period_ticks);

    /* 确保 CCR 预装载开启，避免 DMA 写入破坏当前 PWM 周期，spec §6 */
    __HAL_TIM_ENABLE_OCxPRELOAD(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL);

    Ws2812_HwStop();
    BspWs2812_Clear();

    ws2812_state = WS2812_STATE_IDLE;
    ws2812_busy = 0u;
}

HAL_StatusTypeDef BspWs2812_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    return BspWs2812_SetColorIndex(0u, red, green, blue);
}

HAL_StatusTypeDef BspWs2812_SetColorIndex(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= BSP_WS2812_LED_NUM)
    {
        return HAL_ERROR;
    }
    if (ws2812_busy != 0u)
    {
        /* spec §11：发送期间禁止修改当前发送显存 */
        return HAL_BUSY;
    }

    ws2812_color[index].red = red;
    ws2812_color[index].green = green;
    ws2812_color[index].blue = blue;

    return HAL_OK;
}

HAL_StatusTypeDef BspWs2812_Show(void)
{
    if (ws2812_busy != 0u)
    {
        /* spec §27 简单策略：忙时直接返回 */
        return HAL_BUSY;
    }

    /* 停止可能残留的传输，清理硬件与 HAL 状态 */
    Ws2812_HwStop();
    __HAL_TIM_CLEAR_FLAG(&WS2812_TIM_HANDLE, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC1OF);

    /* 初始化游程，spec §13 */
    ws2812_led_index = 0u;
    ws2812_bit_index = 0u;
    ws2812_preamble_remain = WS2812_PREAMBLE_SLOTS;
    ws2812_reset_remain = ws2812_reset_total;
    ws2812_state = WS2812_STATE_DATA;

    /* 启动前预填充两个半区，spec §19 */
    Ws2812_FillHalf(WS2812_HALF_FIRST);
    Ws2812_FillHalf(WS2812_HALF_SECOND);

    ws2812_busy = 1u;

    /* 标准接口启动，spec §20 */
    if (HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE,
                              WS2812_TIM_CHANNEL,
                              ws2812_dma_buf,
                              WS2812_FULL_BUFFER_SLOTS) != HAL_OK)
    {
        Ws2812_HwStop();
        ws2812_state = WS2812_STATE_ERROR;
        ws2812_busy = 0u;
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BspWs2812_WriteColor(uint8_t red, uint8_t green, uint8_t blue)
{
    HAL_StatusTypeDef status;

    status = BspWs2812_SetColor(red, green, blue);
    if (status != HAL_OK)
    {
        return status;
    }

    return BspWs2812_Show();
}

void BspWs2812_Clear(void)
{
    uint16_t index;

    if (ws2812_busy != 0u)
    {
        return;
    }

    for (index = 0u; index < BSP_WS2812_LED_NUM; index++)
    {
        ws2812_color[index].red = 0u;
        ws2812_color[index].green = 0u;
        ws2812_color[index].blue = 0u;
    }
}

uint8_t BspWs2812_IsBusy(void)
{
    return ws2812_busy;
}
