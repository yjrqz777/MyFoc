# ADC 注入组与 PWM 采样触发时序（谷点采样方案）

## 1. 目标

ADC1 注入组由 TIM1 CH4 比较事件触发，用于采集三相低侧采样电阻电流。采样点调整为：

> TIM1 向上计数期间，从 CNT=0 起延时一小段可配置时间后触发 ADC1 注入组转换。

采样时序约束：

```
samplePoint + adcTime + margin < min(CCR_A, CCR_B, CCR_C)
```

即 ADC 采样保持必须在最小相 CCR 之前完成，确保采样期间三相低侧均导通。

## 2. PWM 模式与采样窗口

TIM1 CH1~CH3 使用 **PWM 模式 2** + 中心对齐模式 2：

- CNT < CCR 时：OCxREF=0 → 高侧关闭 → **低侧导通**（互补输出有效）
- CNT >= CCR 时：OCxREF=1 → 高侧导通 → 低侧关闭

因此在 CNT=0（谷点）附近，三相 CNT < CCR 同时成立，三相低侧公共导通（零矢量 000），
形成采样窗口。

## 3. 当前相关参数

| 参数 | 数值 |
| --- | ---: |
| TIM1 时钟 | 170 MHz |
| TIM1 预分频 | 0 |
| ARR | 8500 |
| PWM 模式 | PWM 模式 2 |
| 计数模式 | 中心对齐模式 2 |
| PWM 频率 | 10 kHz |
| 完整 PWM 周期 | 100 us |
| 半周期 | 50 us |
| ADC 触发 CCR4 | BSP_PWM_ADC_SAMPLE_POINT_TICKS = 500 |
| CCR 下限 | BSP_PWM_ADC_CCR_MIN = 900 |
| 电压标幺值→CCR 公式 | CCR = PWM/2 - (V/100) * PWM/2 |

定时器计数值与时间换算：

```text
1 tick = 1 / 170 MHz ≈ 5.88 ns
500 ticks ≈ 2.9 us
900 ticks ≈ 5.3 us
```

## 4. 中心对齐计数示意

```text
CNT
8500                                      /\
                                         /  \
                                        /    \
                                高侧导通区间  高侧导通区间
                              /                \
5100  ------------------*                      *------  CCR=5100 (V=-20%)
                        |                      |
4250  ------------------|----------------------|------  CCR=4250 (V=0, 零电压)
                        |                      |
3400  ------------------*                      *------  CCR=3400 (V=+20%, FOC调制上限)
                        |    低侧导通区间       |
                        |                      |
900   ----------*-------|                      |-------*------  CCR_MIN
               |        |                      |       |
500   ----*----|        |                      |       |
          ↑    |        |                      |       |
      ADC 触发 |        |                      |       |
               |        |                      |       |
   0  *--------+--------+----------------------+-------+--------*
      0       2.9us    20us     25us          30us    52.9us   50us    100us
          向上计数 ─────────────→│←──────────────── 向下计数
                                  ARR

↑ 谷点采样窗口：CNT ∈ [0, min(CCR_A, CCR_B, CCR_C))
  低侧全开，ADC 在 CCR4=500 处触发采样
```

TIM1 使用中心对齐模式 2，CH4 比较事件仅在向上计数时产生。因此当 CNT 向上计数到
CCR4=500 时，硬件触发 ADC1 注入组。

## 5. 低侧 MOSFET 与采样时序

以最小相 CCR 为例（CCR=3400，对应 FOC 最大正电压 V=+20）：

```text
时间：        0 us        2.9us                     20us         25us       100us
              │            │                          │            │            │

TIM1 CNT：    0 ────────→ 500 ────────────────────→ 3400 ──────→ 4250 ──────→ ...
                           ↑                          ↑
                       ADC 采样                    该相低侧关断
                                                  (CNT >= CCR, 高侧导通)

低侧 MOSFET： │█████████████████████████████████████│
              └────────── 导通区间 (CNT < CCR) ──────┘
              ↑          ↑
         CNT=0 谷点   ADC 采样点
         低侧已导通   采样保持结束 (500+200+200=900 < 3400) ✓
```

谷点低侧导通时间估算：
- 向下计数越过 CCR 时低侧开启，到 CNT=0 已导通 CCR 个计数
- 向上计数从 0 到 CCR4=500，低侧继续导通
- 总导通时间 = CCR + 500 >= 3400 + 500 = 3900 ticks ≈ 23 us，远大于运放建立时间

采样约束验证：
```
samplePoint + adcTime + margin = 500 + 200 + 200 = 900
min(CCR_A, CCR_B, CCR_C) >= 3400 (FOC 调制限幅 V_max=20)
900 < 3400 ✓
```

## 6. ADC 转换链路

```text
TIM1 向上计数到 CCR4=500
              │
              ▼
       TIM1 CH4 比较事件 (仅向上计数)
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

## 7. 对应代码配置

CubeMX/TIM1 配置：

```c
htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
sConfigOC.OCMode = TIM_OCMODE_PWM2;  /* PWM 模式 2：CNT<CCR 时低侧导通 */
```

ADC 触发位置（bsp_pwm.c）：

```c
#define BSP_PWM_ADC_SAMPLE_POINT_TICKS   500u  /* 从 CNT=0 起延时 */
#define BSP_PWM_ADC_CONV_TIME_TICKS      200u  /* ADC 4 通道转换时间 */
#define BSP_PWM_ADC_MARGIN_TICKS         200u  /* 安全裕量 */
#define BSP_PWM_ADC_CCR_MIN  (500 + 200 + 200)  /* CCR 下限 = 900 */
```

运行时设置：

```c
CCR4 = BSP_PWM_ADC_SAMPLE_POINT_TICKS;  /* = 500 */
```

电压标幺值转 CCR（PWM 模式 2，占空比反向）：

```c
CCR = PWM/2 - (V/100) * PWM/2;
/* V=+20 → CCR=3400, V=0 → CCR=4250, V=-20 → CCR=5100 */
/* CCR 下限限制为 900，确保采样窗口 */
```

## 8. 启动时刻说明

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

## 9. CubeMX 同步说明

`tim.c` 中 `sConfigOC.OCMode` 已从 `TIM_OCMODE_PWM1` 改为 `TIM_OCMODE_PWM2`。
若在 CubeMX 中重新生成代码，需在 TIM1 的 Channel1/2/3 配置页将 PWM Mode 改为
"PWM Mode 2"，否则会被覆盖回 PWM Mode 1。
