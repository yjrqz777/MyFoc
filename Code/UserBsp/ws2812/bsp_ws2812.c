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
#define WS2812_BITS_PER_LED         (24u)
/* 每半区容纳 LED 数量，spec §9 推荐 4 */
#define WS2812_LEDS_PER_HALF        (4u)
#define WS2812_HALF_BUFFER_SLOTS    (WS2812_LEDS_PER_HALF * WS2812_BITS_PER_LED)   /* 96  */
#define WS2812_FULL_BUFFER_SLOTS    (WS2812_HALF_BUFFER_SLOTS * 2u)                /* 192 */

/* 帧首前导低电平 Slot 数量，spec §21 建议 2~8，用于稳定 DMA/Preload 流水线 */
#define WS2812_PREAMBLE_SLOTS       (4u)

/* 帧尾 Reset 时间，spec §15 默认 300us，可配置 */
#define WS2812_RESET_TIME_US        (300u)

/* TIM2 计数时钟：G474 下 APB 定时器时钟 = 170MHz (PSC=0) */
#define WS2812_TIM_COUNTER_CLK_HZ   (170000000u)

/* CCR 占空比比例，spec §3：Code0 ≈ 1/3，Code1 ≈ 2/3。需用示波器校准。 */
#define WS2812_CODE0_RATIO_NUM      (1u)
#define WS2812_CODE0_RATIO_DEN      (3u)
#define WS2812_CODE1_RATIO_NUM      (2u)
#define WS2812_CODE1_RATIO_DEN      (3u)

/* TIM2 CH1 对应 ChannelState 数组下标 */
#define WS2812_CHANNEL_STATE_INDEX  (0u)

/*---------------------- 状态机 ----------------------*/
typedef enum
{
    E_WS2812_STATE_IDLE = 0u,
    E_WS2812_STATE_DATA,         /* 将 RGB 编码为 CCR */
    E_WS2812_STATE_RESET,        /* 输出帧尾 Reset 低电平 */
    E_WS2812_STATE_DRAIN,        /* 等待最后半区发送完成后再停止 */
    E_WS2812_STATE_STOPPING,
    E_WS2812_STATE_ERROR,
} eBspWs2812StateDef;

/* 半区编号：HT 回调可安全修改前半区，TC 回调可安全修改后半区 */
typedef enum
{
    E_WS2812_HALF_FIRST = 0u,    /* 前半区 */
    E_WS2812_HALF_SECOND = 1u,   /* 后半区 */
} eBspWs2812HalfDef;

/*---------------------- 运行变量 ----------------------*/
static tBspWs2812ColorDef tWs2812Color[BSP_WS2812_LED_NUM];
static uint32_t u32Ws2812DmaBuffer[WS2812_FULL_BUFFER_SLOTS];

static volatile eBspWs2812StateDef eWs2812State = E_WS2812_STATE_IDLE;
static volatile uint8_t u8Ws2812Busy = 0u;

static uint16_t u16Ws2812LedIndex;        /* 当前编码的 LED 索引 */
static uint8_t u8Ws2812BitIndex;          /* 当前 LED 内 bit 位 0..23 */
static uint16_t u16Ws2812PreambleRemain;  /* 帧首前导低电平剩余 Slot */
static uint16_t u16Ws2812ResetRemain;     /* 帧尾 Reset 剩余 Slot */
static uint16_t u16Ws2812ResetTotal;      /* 帧尾 Reset 总 Slot 数 */
static eBspWs2812HalfDef eWs2812FinalHalf; /* 最后 Reset 数据所在半区 */

static uint32_t u32Ws2812CcrCode0;
static uint32_t u32Ws2812CcrCode1;

/*---------------------- 应用回调（弱定义） ----------------------*/
__weak void BspWs2812OnComplete(void)
{
}

__weak void BspWs2812OnError(void)
{
}

/*---------------------- 底层停止 ----------------------*/
/*
 * 直接寄存器操作停止 TIM+DMA，可在 ISR 中安全调用。
 * 不依赖 HAL_TIM_PWM_Stop_DMA（其在 ISR 中调用 HAL_DMA_Abort_IT 会遗留
 * ABORT 状态），手动复位 HAL 状态以保证下次 HAL_TIM_PWM_Start_DMA 可用。
 */
static void BspWs2812HwStop(void)
{
    DMA_HandleTypeDef * Dma;

    /* 关闭 TIM DMA 请求、PWM 通道输出、定时器 */
    __HAL_TIM_DISABLE_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_DMA_ENABLE_BIT);
    WS2812_TIM_HANDLE.Instance->CCER &= ~TIM_CCER_CC1E;
    __HAL_TIM_DISABLE(&WS2812_TIM_HANDLE);

    /* 关闭并复位 DMA 通道（DMA1_Channel2） */
    Dma = WS2812_TIM_HANDLE.hdma[WS2812_TIM_DMA_ID];
    if (Dma != NULL)
    {
        __HAL_DMA_DISABLE(Dma);
        Dma->State = HAL_DMA_STATE_READY;
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
 * 半区边界可能落在某颗 LED 中间，通过 led index/bit index 游程断点续编。
 */
static void BspWs2812FillHalf(eBspWs2812HalfDef eHalf)
{
    uint16_t Offset = (eHalf == E_WS2812_HALF_FIRST) ? 0u : WS2812_HALF_BUFFER_SLOTS;
    uint16_t Remain = WS2812_HALF_BUFFER_SLOTS;

    while (Remain > 0u)
    {
        switch (eWs2812State)
        {
            case E_WS2812_STATE_DATA:
                if (u16Ws2812PreambleRemain > 0u)
                {
                    /* 帧首前导低电平，稳定 DMA/Preload 流水线，spec §21 */
                    u32Ws2812DmaBuffer[Offset] = 0u;
                    u16Ws2812PreambleRemain--;
                }
                else if (u16Ws2812LedIndex < BSP_WS2812_LED_NUM)
                {
                    /* 编码一位：GRB 顺序，MSB First */
                    uint8_t ByteSelect = (uint8_t)(u8Ws2812BitIndex / 8u);  /* 0=G, 1=R, 2=B */
                    uint8_t BitInByte = (uint8_t)(u8Ws2812BitIndex % 8u);
                    uint8_t ColorByte;
                    uint8_t Mask;

                    if (ByteSelect == 0u)
                    {
                        ColorByte = tWs2812Color[u16Ws2812LedIndex].u8Green;
                    }
                    else if (ByteSelect == 1u)
                    {
                        ColorByte = tWs2812Color[u16Ws2812LedIndex].u8Red;
                    }
                    else
                    {
                        ColorByte = tWs2812Color[u16Ws2812LedIndex].u8Blue;
                    }
                    Mask = (uint8_t)(0x80u >> BitInByte);
                    u32Ws2812DmaBuffer[Offset] =
                        ((ColorByte & Mask) != 0u) ? u32Ws2812CcrCode1 : u32Ws2812CcrCode0;

                    u8Ws2812BitIndex++;
                    if (u8Ws2812BitIndex >= WS2812_BITS_PER_LED)
                    {
                        u8Ws2812BitIndex = 0u;
                        u16Ws2812LedIndex++;
                    }
                }
                else
                {
                    /* 所有 LED 数据编码完成，切换到 RESET，不消耗 Slot */
                    eWs2812State = E_WS2812_STATE_RESET;
                    continue;
                }
                break;

            case E_WS2812_STATE_RESET:
                if (u16Ws2812ResetRemain > 0u)
                {
                    u32Ws2812DmaBuffer[Offset] = 0u;
                    u16Ws2812ResetRemain--;
                }
                else
                {
                    /* 最后 Reset 已写入，记录所在半区并进入 DRAIN，不消耗 Slot */
                    eWs2812FinalHalf = eHalf;
                    eWs2812State = E_WS2812_STATE_DRAIN;
                    continue;
                }
                break;

            case E_WS2812_STATE_DRAIN:
            case E_WS2812_STATE_STOPPING:
            case E_WS2812_STATE_ERROR:
            default:
                /* 继续填充低电平，保证输出安全 */
                u32Ws2812DmaBuffer[Offset] = 0u;
                break;
        }

        Offset++;
        Remain--;
    }
}

/*---------------------- 完成与错误处理 ----------------------*/
static void BspWs2812CompleteStop(void)
{
    BspWs2812HwStop();
    eWs2812State = E_WS2812_STATE_IDLE;
    u8Ws2812Busy = 0u;
    BspWs2812OnComplete();
}

static void BspWs2812GotoError(void)
{
    BspWs2812HwStop();
    eWs2812State = E_WS2812_STATE_ERROR;
    u8Ws2812Busy = 0u;
    BspWs2812OnError();
}

/*---------------------- HAL PWM DMA 回调 ----------------------*/
/*
 * 半传输完成：前半区刚发送完，DMA 正在发送后半区。
 * 可安全修改前半区。若最后 Reset 所在半区为前半区且已发送完毕则停止。
 */
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef * ptHtim)
{
    if (ptHtim != &WS2812_TIM_HANDLE)
    {
        return;
    }
    if (u8Ws2812Busy == 0u)
    {
        return;
    }

    if ((eWs2812State == E_WS2812_STATE_DRAIN) &&
        (eWs2812FinalHalf == E_WS2812_HALF_FIRST))
    {
        BspWs2812CompleteStop();
    }
    else
    {
        BspWs2812FillHalf(E_WS2812_HALF_FIRST);
    }
}

/*
 * 传输完成（Circular）：后半区刚发送完，DMA 回到前半区。
 * 可安全修改后半区。若最后 Reset 所在半区为后半区且已发送完毕则停止。
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * ptHtim)
{
    if (ptHtim != &WS2812_TIM_HANDLE)
    {
        return;
    }
    if (u8Ws2812Busy == 0u)
    {
        return;
    }

    if ((eWs2812State == E_WS2812_STATE_DRAIN) &&
        (eWs2812FinalHalf == E_WS2812_HALF_SECOND))
    {
        BspWs2812CompleteStop();
    }
    else
    {
        BspWs2812FillHalf(E_WS2812_HALF_SECOND);
    }
}

/* DMA 错误回调 */
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef * ptHtim)
{
    if (ptHtim != &WS2812_TIM_HANDLE)
    {
        return;
    }
    BspWs2812GotoError();
}

/*---------------------- 公开接口 ----------------------*/
void BspWs2812Init(void)
{
    uint32_t PeriodTicks = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&WS2812_TIM_HANDLE) + 1u;

    /* 计算码值，spec §3 */
    u32Ws2812CcrCode0 = (PeriodTicks * WS2812_CODE0_RATIO_NUM) / WS2812_CODE0_RATIO_DEN;
    u32Ws2812CcrCode1 = (PeriodTicks * WS2812_CODE1_RATIO_NUM) / WS2812_CODE1_RATIO_DEN;

    /* Reset Slot 数量 = ceil(RESET_TIME_US * counter_clk / period_ticks) */
    u16Ws2812ResetTotal = (uint16_t)(((uint32_t)WS2812_RESET_TIME_US *
                                     (WS2812_TIM_COUNTER_CLK_HZ / 1000000u) +
                                     PeriodTicks - 1u) / PeriodTicks);

    /* 确保 CCR 预装载开启，避免 DMA 写入破坏当前 PWM 周期，spec §6 */
    __HAL_TIM_ENABLE_OCxPRELOAD(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL);

    BspWs2812HwStop();
    BspWs2812Clear();

    eWs2812State = E_WS2812_STATE_IDLE;
    u8Ws2812Busy = 0u;
}

HAL_StatusTypeDef BspWs2812SetColor(uint8_t u8Red, uint8_t u8Green, uint8_t u8Blue)
{
    return BspWs2812SetColorIndex(0u, u8Red, u8Green, u8Blue);
}

HAL_StatusTypeDef BspWs2812SetColorIndex(uint16_t u16Index,
                                        uint8_t u8Red,
                                        uint8_t u8Green,
                                        uint8_t u8Blue)
{
    if (u16Index >= BSP_WS2812_LED_NUM)
    {
        return HAL_ERROR;
    }
    if (u8Ws2812Busy != 0u)
    {
        /* spec §11：发送期间禁止修改当前发送显存 */
        return HAL_BUSY;
    }

    tWs2812Color[u16Index].u8Red = u8Red;
    tWs2812Color[u16Index].u8Green = u8Green;
    tWs2812Color[u16Index].u8Blue = u8Blue;

    return HAL_OK;
}

HAL_StatusTypeDef BspWs2812Show(void)
{
    if (u8Ws2812Busy != 0u)
    {
        /* spec §27 简单策略：忙时直接返回 */
        return HAL_BUSY;
    }

    /* 停止可能残留的传输，清理硬件与 HAL 状态 */
    BspWs2812HwStop();
    __HAL_TIM_CLEAR_FLAG(&WS2812_TIM_HANDLE, TIM_FLAG_UPDATE | TIM_FLAG_CC1 | TIM_FLAG_CC1OF);

    /* 初始化游程，spec §13 */
    u16Ws2812LedIndex = 0u;
    u8Ws2812BitIndex = 0u;
    u16Ws2812PreambleRemain = WS2812_PREAMBLE_SLOTS;
    u16Ws2812ResetRemain = u16Ws2812ResetTotal;
    eWs2812State = E_WS2812_STATE_DATA;

    /* 启动前预填充两个半区，spec §19 */
    BspWs2812FillHalf(E_WS2812_HALF_FIRST);
    BspWs2812FillHalf(E_WS2812_HALF_SECOND);

    u8Ws2812Busy = 1u;

    /* 标准接口启动，spec §20 */
    if (HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE,
                              WS2812_TIM_CHANNEL,
                              u32Ws2812DmaBuffer,
                              WS2812_FULL_BUFFER_SLOTS) != HAL_OK)
    {
        BspWs2812HwStop();
        eWs2812State = E_WS2812_STATE_ERROR;
        u8Ws2812Busy = 0u;
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BspWs2812WriteColor(uint8_t u8Red, uint8_t u8Green, uint8_t u8Blue)
{
    HAL_StatusTypeDef Status;

    Status = BspWs2812SetColor(u8Red, u8Green, u8Blue);
    if (Status != HAL_OK)
    {
        return Status;
    }

    return BspWs2812Show();
}

void BspWs2812Clear(void)
{
    uint16_t Index;

    if (u8Ws2812Busy != 0u)
    {
        return;
    }

    for (Index = 0u; Index < BSP_WS2812_LED_NUM; Index++)
    {
        tWs2812Color[Index].u8Red = 0u;
        tWs2812Color[Index].u8Green = 0u;
        tWs2812Color[Index].u8Blue = 0u;
    }
}

uint8_t BspWs2812IsBusy(void)
{
    return u8Ws2812Busy;
}
