# PA5 WS2812 PWM DMA 调试总结

## 1. 问题背景

项目使用 STM32G474RE 的 **PA5 / TIM2_CH1** 驱动一颗 WS2812，通过定时器 PWM 和 DMA 发送 24 bit GRB 数据。

调试过程中先后出现以下现象：

1. PA5 没有波形，WS2812 不亮。
2. 恢复波形后，输出时序不符合 WS2812 协议。
3. 发送 `BspWs2812_WriteColor(32u, 32u, 0u)` 时，测得：

```text
约 400 ns 高 + 920 ns 低
约 340 ns 高 + 920 ns 低
随后持续高电平约 27 us
```

该波形的关键特征是：**前两个 bit 基本正常，从第三个 bit（DMA 开始接管 CCR1）起异常。**

---

## 2. 最初无波形的原因

曾在启动 WS2812 DMA 前调用普通 PWM 启动：

```c
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
```

该调用会把 TIM2_CH1 的 HAL 通道状态置为 Busy。之后再调用：

```c
HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, ...);
```

可能返回 `HAL_BUSY`，导致 DMA PWM 没有真正启动。

因此普通固定 PWM 测试代码不能和 WS2812 PWM DMA 同时使用：

```c
// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
// __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 106u);
```

如果需要单独测试固定 PWM，应停止 WS2812 驱动，并先设置 CCR，再启动 PWM：

```c
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 106u);
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
```

---

## 3. TIM2 和 WS2812 的理论时序

TIM2 当前配置：

```text
TIM2 时钟：170 MHz
PSC：0
ARR：211
计数周期：212 tick
```

因此一个 PWM 周期为：

```text
Tbit = 212 / 170 MHz ≈ 1.247 us
频率 ≈ 801.9 kHz
```

### 3.1 发送逻辑 0

当前编码比例为周期的 28%：

```c
CCR1 = (212 * 28) / 100 = 59
```

理论波形：

```text
高电平：59 / 170 MHz ≈ 0.347 us
低电平：1.247 - 0.347 ≈ 0.900 us
```

### 3.2 发送逻辑 1

当前编码比例为周期的 56%：

```c
CCR1 = (212 * 56) / 100 = 118
```

理论波形：

```text
高电平：118 / 170 MHz ≈ 0.694 us
低电平：1.247 - 0.694 ≈ 0.553 us
```

---

## 4. 当前颜色数据和理论波形

启动阶段调用：

```c
BspWs2812_WriteColor(32u, 32u, 0u);
```

接口参数顺序为 RGB：

```text
R = 32 = 0x20
G = 32 = 0x20
B = 0  = 0x00
```

WS2812 在线上传输顺序为 **GRB**，所以发送数据为：

```text
G：00100000
R：00100000
B：00000000
```

完整 24 bit：

```text
00100000 00100000 00000000
```

理论高脉宽序列：

```text
G：短 短 长 短 短 短 短 短
R：短 短 长 短 短 短 短 短
B：短 短 短 短 短 短 短 短
```

其中：

```text
短高脉冲 ≈ 0.347 us，表示 0
长高脉冲 ≈ 0.694 us，表示 1
每个 bit 总周期 ≈ 1.247 us
24 bit 数据总时间 ≈ 29.93 us
```

因此前三个 bit 应为：

```text
bit0 = 0：约 0.347 us 高 + 0.900 us 低
bit1 = 0：约 0.347 us 高 + 0.900 us 低
bit2 = 1：约 0.694 us 高 + 0.553 us 低
```

---

## 5. 前两位正常、随后持续高电平的原因

TIM2 是一个 **32 位定时器**，`TIM2->CCR1` 是 32 位比较寄存器。

异常时 DMA 配置为半字传输：

```c
hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
```

而前两个 bit 是 CPU 通过 32 位寄存器写操作手动装入 CCR1，第三个 bit 开始才由 DMA 写入。因此波形正好表现为：

```text
前两个 bit 正常
第三个 bit 开始异常
```

DMA 以 16 位方式写 32 位 TIM2 CCR1 时，可能使实际比较寄存器值不再是预期的 `59` 或 `118`。一旦 CCR1 的实际值大于 ARR=211，在一个 PWM 周期内就不会发生有效比较，PWM1 输出会持续保持高电平。

修复方式是让缓冲区和 DMA 都使用 32 位：

```c
static uint32_t ws2812_pwm_buf[WS2812_BUFFER_LEN];
```

```c
hdma_tim2_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
hdma_tim2_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
```

CubeMX 配置文件也已同步为：

```text
Dma.TIM2_UP.1.MemDataAlignment=DMA_MDATAALIGN_WORD
Dma.TIM2_UP.1.PeriphDataAlignment=DMA_PDATAALIGN_WORD
```

> 注意：如果后续改回 `TIM2_CH1` 请求，也必须继续使用 `WORD` 数据宽度。

---

## 6. 缓冲区和 DMA 触发源是两个独立概念

“使用缓冲区”表示先生成完整的 CCR 数据数组：

```c
static uint32_t ws2812_pwm_buf[WS2812_BUFFER_LEN];
```

缓冲区通常包含：

```text
24 × LED数量 个颜色数据槽
+ 若干个 CCR=0 的 Reset 槽
```

“DMA 触发源”只决定：**定时器在什么时刻要求 DMA 搬运下一个缓冲区元素。**

所以以下两种方案都属于缓冲区 DMA：

```text
缓冲区 + TIM2_CH1 比较事件触发
缓冲区 + TIM2_UP 更新事件触发
```

---

## 7. 标准 HAL PWM DMA 方式：TIM2_CH1

STM32 HAL 提供的标准 PWM DMA API 为：

```c
HAL_TIM_PWM_Start_DMA(&htim2,
                      TIM_CHANNEL_1,
                      ws2812_pwm_buf,
                      WS2812_BUFFER_LEN);
```

对于 `TIM_CHANNEL_1`，HAL 内部使用：

```text
DMA 句柄槽：TIM_DMA_ID_CC1
定时器 DMA 请求：TIM_DMA_CC1
DMAMUX 请求源：DMA_REQUEST_TIM2_CH1
DMA 目标寄存器：TIM2->CCR1
```

对应配置必须配套：

```c
hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_CH1;
```

```c
__HAL_LINKDMA(tim_pwmHandle,
              hdma[TIM_DMA_ID_CC1],
              hdma_tim2_ch1);
```

启动时使用：

```c
HAL_TIM_PWM_Start_DMA(&htim2,
                      TIM_CHANNEL_1,
                      ws2812_pwm_buf,
                      WS2812_BUFFER_LEN);
```

DMA 完成回调中停止 PWM，并确保 CCR1 和 PA5 回到低电平。

### 7.1 CH1 的触发时机

PWM1 模式下：

```text
CNT < CCR1：输出高电平
CNT >= CCR1：输出低电平
```

CH1 比较事件发生在：

```text
CNT == CCR1
```

也就是 PWM 高电平结束、下降沿附近。

发送 0 时：

```text
CCR1 = 59
DMA 在周期开始约 0.347 us 后触发
```

发送 1 时：

```text
CCR1 = 118
DMA 在周期开始约 0.694 us 后触发
```

开启 CCR 预装载后，DMA 在当前周期下降沿写入下一个 CCR 值，该值在下一次 Update 时生效。

### 7.2 CH1 请求间隔

虽然每个 PWM 周期通常触发一次，但触发位置跟随 CCR1 改变，所以相邻 DMA 请求间隔不是固定值：

| 当前 bit → 下一 bit | DMA 请求间隔 |
|---|---:|
| 0 → 0 | 约 1.247 us |
| 0 → 1 | 约 1.594 us |
| 1 → 0 | 约 0.900 us |
| 1 → 1 | 约 1.247 us |

这不会改变 PWM 的 bit 周期，PWM 周期仍始终由 ARR 决定。

---

## 8. 自定义 Update DMA 方式：TIM2_UP

当前驱动使用的请求源为：

```c
hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_UP;
```

它表示 TIM2 每次产生 Update 事件时触发一次 DMA。

向上计数模式中，Update 发生在：

```text
CNT 从 ARR 重新回到 0
```

因此请求间隔固定为：

```text
212 / 170 MHz ≈ 1.247 us
```

无论相邻数据是：

```text
0 → 0
0 → 1
1 → 0
1 → 1
```

Update DMA 请求间隔都固定为一个 PWM 周期。

Update DMA 配置必须配套：

```c
hdma_tim2_ch1.Init.Request = DMA_REQUEST_TIM2_UP;
```

```c
__HAL_LINKDMA(tim_pwmHandle,
              hdma[TIM_DMA_ID_UPDATE],
              hdma_tim2_ch1);
```

```c
__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
```

当前实现没有直接调用 `HAL_TIM_PWM_Start_DMA()`，而是调用：

```c
HAL_DMA_Start_IT(&hdma_tim2_ch1,
                 (uint32_t)&ws2812_pwm_buf[2],
                 (uint32_t)&TIM2->CCR1,
                 WS2812_BUFFER_LEN - 2u);
```

### 8.1 Update DMA 的预装载流水线

CCR1 开启预装载后，Update 事件会先把 preload CCR 装入 active CCR。由同一个 Update 事件触发的 DMA 写入，会成为后续的 preload 值，要等下一次 Update 才生效。

所以当前实现预先装入两个 bit：

```text
active CCR：bit0
preload CCR：bit1
DMA首地址：bit2
```

执行过程：

```text
周期0输出 bit0

Update1：
    bit1进入active
    DMA把bit2写入preload

周期1输出 bit1

Update2：
    bit2进入active
    DMA把bit3写入preload
```

---

## 9. 两种 DMA 触发方式对比

| 项目 | TIM2_CH1 | TIM2_UP |
|---|---|---|
| 触发事件 | `CNT == CCR1` | `CNT` 从 ARR 回到 0 |
| 触发位置 | PWM下降沿 | PWM周期边界 |
| 请求间隔 | 随相邻占空比变化 | 固定一个PWM周期 |
| 是否使用完整缓冲区 | 是 | 是 |
| 是否直接适配 `HAL_TIM_PWM_Start_DMA()` | 是 | 否，需要自定义启动 |
| DMA关联槽 | `TIM_DMA_ID_CC1` | `TIM_DMA_ID_UPDATE` |
| 定时器DMA使能 | `TIM_DMA_CC1` | `TIM_DMA_UPDATE` |
| CCR大于ARR时 | 可能不再产生比较请求，DMA停住 | Update请求仍会继续 |
| 实现复杂度 | 较低 | 较高，需要预装流水线 |

---

## 10. 推荐方案

如果目标是：

> 预先构建完整 WS2812 PWM 缓冲区，然后通过一个 HAL API 一次发送完成。

推荐使用标准 HAL PWM DMA 方式：

```text
DMA_REQUEST_TIM2_CH1
+ TIM_DMA_ID_CC1
+ TIM_DMA_CC1
+ HAL_TIM_PWM_Start_DMA()
```

如果希望：

> DMA 请求严格固定在每个 PWM 周期边界，并自行控制 active/preload 流水线。

可以继续使用当前自定义 Update DMA：

```text
DMA_REQUEST_TIM2_UP
+ TIM_DMA_ID_UPDATE
+ TIM_DMA_UPDATE
+ HAL_DMA_Start_IT()
```

无论选择哪一种方案，都必须继续保证：

```text
TIM2 CCR1 使用32位数据
ws2812_pwm_buf 使用 uint32_t
DMA内存宽度使用 WORD
DMA外设宽度使用 WORD
普通 HAL_TIM_PWM_Start() 不与 PWM DMA 同时启动
```

---

## 11. 当前仓库实现状态

截至本次整理，当前代码采用：

```text
PA5 / TIM2_CH1 PWM输出
TIM2_UP 作为DMA请求源
CCR1预装载
bit0和bit1手动预装
DMA从bit2开始发送
uint32_t PWM缓冲区
DMA内存和外设宽度均为WORD
```

如果后续决定改为标准 HAL PWM DMA，需要同步修改以下内容，不能只修改请求源一行：

1. `DMA_REQUEST_TIM2_UP` 改为 `DMA_REQUEST_TIM2_CH1`。
2. `TIM_DMA_ID_UPDATE` 改为 `TIM_DMA_ID_CC1`。
3. `TIM_DMA_UPDATE` 改为 `TIM_DMA_CC1`。
4. 自定义 `HAL_DMA_Start_IT()` 改为 `HAL_TIM_PWM_Start_DMA()`。
5. 调整发送完成回调和输出停止逻辑。
6. 保持缓冲区与 DMA 宽度为 32 位。
