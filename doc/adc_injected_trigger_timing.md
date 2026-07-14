# ADC 注入组与 PWM 采样触发时序

## 1. 目标

ADC1 注入组由 TIM1 CH4 比较事件触发，用于采集三相低侧采样电阻电流。采样点调整为：

> TIM1 向上计数期间，等待三相低侧 MOSFET 全部开启至少 10 us 后，触发 ADC1 注入组转换。

这里的“PWM 开启”是指低侧 MOSFET 导通，不是调用 `BspPwm_StartPowerOutputs()` 的软件时刻，也不是高侧 MOSFET 导通时刻。

## 2. 当前相关参数

| 参数 | 数值 |
| --- | ---: |
| TIM1 时钟 | 170 MHz |
| TIM1 预分频 | 0 |
| ARR | 8500 |
| PWM 模式 | 中心对齐模式 2 |
| PWM 频率 | 10 kHz |
| 完整 PWM 周期 | 100 us |
| 半周期 | 50 us |
| 相 PWM 最大 CCR | 60% ARR = 5100 |
| ADC 触发 CCR4 | 80% ARR = 6800 |

10 us 对应的 TIM1 计数值：

```text
170 MHz × 10 us = 1700
```

最晚开启的低侧 MOSFET 对应最大相 CCR：

```text
CCR_phase_max = 8500 × 60% = 5100
```

因此 ADC 触发点为：

```text
CCR4 = CCR_phase_max + 1700
     = 5100 + 1700
     = 6800
     = 80% ARR
```

## 3. 中心对齐计数示意

```text
CNT
8500                                      /\
                                         /  \
                                        /    \
6800  ---------------------------------*      \----------------  CCR4
                                      ↑
                                  ADC 注入触发
                                     40 us

5100  --------------------------*              *---------------  最大相 CCR
                              ↑                  ↑
                     最后一路低侧开启       第一路低侧关闭
                            30 us               70 us

   0  *---------------------------------------------------------*
      0                  30  40     50        70                100 us
          向上计数 ───────────────→│←──────────────── 向下计数
                                  ARR
```

TIM1 使用中心对齐模式 2，CH4 比较事件选择在向上计数阶段产生。因此当 CNT 向上计数到 CCR4=6800 时，硬件触发 ADC1 注入组。

## 4. 低侧 MOSFET 与采样时序

以最晚开启的一相为例，该相 CCR=5100：

```text
时间：        0 us             30 us         40 us       50 us       70 us       100 us
              │                 │             │           │           │             │

TIM1 CNT：    0 ─────────────→ 5100 ───────→ 6800 ─────→ 8500 ─────→ 5100 ───────→ 0

高侧 MOSFET： █████████████████│                                          │████████████
                               └────────────── 关闭 ──────────────────────┘

低侧 MOSFET：                  │██████████████████████████████████████████│
                              └────────────── 导通区间 ──────────────────┘
                               │             ↑
                               │          ADC 采样
                               └── 10 us ────┘
```

实际桥臂切换还包含死区和开关瞬态：

```text
高侧关闭 → 死区 → 低侧开启 → 等待至少 10 us → ADC 采样
```

不在开关边沿立即采样，可以避开死区、MOSFET 开关振铃以及电流放大器建立过程。

## 5. ADC 转换链路

```text
TIM1 向上计数到 CCR4=6800
              │
              ▼
       TIM1 CH4 比较事件
              │
              ▼
      ADC1 注入组硬件触发
              │
              ├── Rank1：Ia
              ├── Rank2：Ib
              ├── Rank3：Ic
              └── Rank4：Ibus
              │
              ▼
       ADC1_2_IRQHandler()
              │
              ▼
HAL_ADCEx_InjectedConvCpltCallback()
              │
              ▼
      UserMotor_FastLoop()
```

ADC1 仍使用以下外部触发配置：

```c
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
```

## 6. 对应代码配置

CubeMX/TIM1 配置：

```c
htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
```

ADC 触发位置：

```c
#define BSP_PWM_ADC_TRIGGER_NUMERATOR    8u
#define BSP_PWM_ADC_TRIGGER_DENOMINATOR  10u
```

运行时计算：

```c
CCR4 = ARR × 8 / 10 = 8500 × 8 / 10 = 6800;
```

如果重新启用 TIM1 CH4 的 PB10 调试翻转代码，向上计数方向判断应为：

```c
((TIM1->CR1 & TIM_CR1_DIR) == 0u)
```

PB10 的软件翻转只用于示波器调试；ADC 是由 TIM1 CH4 硬件事件直接触发，不依赖 PB10 或 TIM1 中断中的 GPIO 操作。

## 7. 启动时刻说明

当前电机启动流程先启动 ADC 注入采样和 TIM1 CH4，再进行静态零偏校准，之后才开启三相功率输出：

```text
BspAdc_StartInjected()
        ↓
BspPwm_StartAdcTrigger()
        ↓
关闭功率桥进行零偏校准
        ↓
BspPwm_StartPowerOutputs()
```

因此本文所述的“10 us”是每个 PWM 周期内，从最晚一路低侧 MOSFET 开启到 ADC 采样点的最小间隔，不表示 `BspPwm_StartPowerOutputs()` 返回后固定 10 us 才进行第一次 ADC 转换。