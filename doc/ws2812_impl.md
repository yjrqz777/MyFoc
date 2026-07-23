# WS2812 驱动实现说明

## 1. 硬件配置

| 项目 | 配置 |
|------|------|
| MCU | STM32G474RE |
| 引脚 | PA5 / TIM2_CH1 |
| TIM2 时钟 | 170 MHz |
| PSC / ARR | 0 / 211（周期 212 tick ≈ 1.247us，约 801.9 kHz） |
| DMA 通道 | DMA1_Channel2 |
| DMA 请求源 | `DMA_REQUEST_TIM2_CH1`（CC1 比较事件触发） |
| DMA 模式 | Circular（环形） |
| DMA 数据宽度 | Word（32 位，匹配 TIM2 的 32 位 CCR1） |

## 2. 架构

```
TIM2 PWM (800kHz)
  └─ CC1 比较事件 → DMA 请求
       └─ Circular DMA 双半区小缓冲区
            ├─ 前半区 (96 CCR)  ← HT 回调填充
            └─ 后半区 (96 CCR)  ← TC 回调填充
```

CPU 只在 DMA 半传输/传输完成时进中断（约每 120us 一次），不逐位进中断。
DMA 缓冲区固定 192 个 CCR，不随灯珠数量增大。

## 3. 关键参数

| 参数 | 值 | 说明 |
|------|----|------|
| `LEDS_PER_HALF` | 4 | 每半区容纳 LED 数 |
| `HALF_BUFFER_SLOTS` | 96 | 每半区 CCR 数（4×24） |
| `FULL_BUFFER_SLOTS` | 192 | 完整 DMA 缓冲区 |
| `PREAMBLE_SLOTS` | 4 | 帧首前导低电平，稳定流水线 |
| `RESET_TIME_US` | 300 | 帧尾 Reset 时间，可配置 |
| Reset Slot 数 | 241 | `ceil(300us / 1.247us)`，Init 时按 ARR 计算 |
| CCR_CODE_0 | 70 | 周期 1/3，需示波器校准 |
| CCR_CODE_1 | 141 | 周期 2/3，需示波器校准 |
| CCR_RESET | 0 | 低电平 |

## 4. 状态机

```
IDLE → DATA → RESET → DRAIN → IDLE
                                    ↘ ERROR
```

| 状态 | 职责 |
|------|------|
| IDLE | 空闲，可修改 RGB 显存或启动 Show |
| DATA | 前导低电平 + RGB 按 GRB/MSB First 编码为 CCR |
| RESET | 输出 CCR=0 帧尾低电平，消耗 Reset Slot |
| DRAIN | 最后 Reset 已写入，等待该半区真正发送完成 |
| ERROR | DMA 错误或状态异常，停止输出 |

## 5. 数据流

一次完整发送的逻辑序列：

```
前导低电平 (4 Slot)
+ LED0 的 24 bit GRB 数据
+ LED1 的 24 bit GRB 数据
+ …
+ 帧尾 Reset 低电平 (≥241 Slot)
+ 额外安全低电平
```

- RGB 显存仅保存颜色（`BspWs2812_Color_t`），发送时才展开为 CCR。
- 半区边界可能落在某颗 LED 中间，通过 `led_index` / `bit_index` 游程断点续编。

## 6. 停止时机（DRAIN）

这是最容易出错的部分。必须区分"数据已写入缓冲区"与"DMA 已实际输出"。

1. 在某半区填入最后 Reset 数据时，记录该半区编号 `final_half`。
2. 进入 DRAIN 状态，继续填 0。
3. 等待 `final_half` 对应的回调再次触发：
   - 最后 Reset 在前半区 → 等下一次 **HT** 回调 → 停止
   - 最后 Reset 在后半区 → 等下一次 **TC** 回调 → 停止
4. 停止后 CCR=0，输出保持低电平。

## 7. HAL 回调

| HAL 回调 | 触发时机 | 处理 |
|----------|----------|------|
| `HAL_TIM_PWM_PulseFinishedHalfCpltCallback` | 前半区发送完（HT） | 填充前半区，或 DRAIN 完成则停止 |
| `HAL_TIM_PWM_PulseFinishedCallback` | 后半区发送完（TC） | 填充后半区，或 DRAIN 完成则停止 |
| `HAL_TIM_ErrorCallback` | DMA 错误 | 停止输出，进入 ERROR |

回调中均检查 `htim == &htim2` 和 `busy` 标志，避免误触发。

## 8. 启动流程（Show）

1. 检查非 Busy
2. `Ws2812_HwStop()` 清理残留状态 + 复位 HAL 状态
3. 清 TIM 标志
4. 初始化游程（led_index、bit_index、preamble、reset 计数）
5. 预填充前半区 + 后半区
6. `HAL_TIM_PWM_Start_DMA()` 启动

> 预填充阶段就可能进入 RESET/DRAIN（灯珠少时），属合法情况。

## 9. 停止方式

ISR 中用直接寄存器操作停止（不调用 `HAL_TIM_PWM_Stop_DMA`，因其内部 `HAL_DMA_Abort_IT` 会遗留 ABORT 状态）：

- 关闭 TIM DMA 请求（`TIM_DMA_CC1`）
- 关闭 PWM 通道输出（`CCER_CC1E`）
- 关闭定时器
- 关闭 DMA 通道，清除 DMA 标志（`DMA_IFCR_CGIF2`）
- CCR=0，CNT=0
- 手动复位 HAL TIM/DMA 状态为 READY

## 10. API

| 接口 | 说明 |
|------|------|
| `BspWs2812_Init()` | 计算 CCR/Reset Slot，确保输出为低 |
| `BspWs2812_SetColor(r,g,b)` | 设置第 0 颗 LED 颜色 |
| `BspWs2812_SetColorIndex(idx,r,g,b)` | 设置指定 LED 颜色 |
| `BspWs2812_Show()` | 异步启动发送，返回 OK/BUSY/ERROR |
| `BspWs2812_WriteColor(r,g,b)` | SetColor + Show 便捷接口 |
| `BspWs2812_Clear()` | 清空 RGB 显存 |
| `BspWs2812_IsBusy()` | 查询发送状态 |
| `BspWs2812_OnComplete()` | 发送完成回调（弱定义） |
| `BspWs2812_OnError()` | 错误回调（弱定义） |

## 11. 与旧实现的主要差异

| 项目 | 旧实现 | 新实现 |
|------|--------|--------|
| DMA 请求源 | `TIM2_UP`（更新事件） | `TIM2_CH1`（CC1 比较事件） |
| DMA 模式 | Normal | Circular |
| 启动接口 | `HAL_DMA_Start_IT()` 自定义 | `HAL_TIM_PWM_Start_DMA()` 标准 |
| 缓冲区 | 整帧展开（随灯数增大） | 固定 192 CCR 双半区 |
| 中断频率 | 仅 TC 一次 | HT/TC 交替（约每 120us） |
| 状态机 | 无 | IDLE/DATA/RESET/DRAIN/ERROR |
| Reset 处理 | 固定 64 Slot | 可配置，默认 300us |
| 前导低电平 | 无（靠预装 2 bit） | 4 个 Slot |
| 停止时机 | TC 立即停 | DRAIN 等半区发送完 |
| 完成回调 | 无 | `BspWs2812_OnComplete` |

## 12. 验证要点

- PWM 周期约 1.25us，Code0/Code1 高电平宽度
- GRB 顺序、MSB First
- 1/2/3/4/5/7/8/9 颗灯的半区边界
- 帧尾 Reset 持续低电平 ≥300us
- 第一位无错位/重复/丢失
- 发送期间运行其他中断不导致旧半区重复
