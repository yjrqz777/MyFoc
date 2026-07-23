# STM32G474 FOC 电流采样与偏置校准方案

> 适用环境：STM32CubeMX + HAL 库  
> 目标芯片：STM32G474  
> 采样方式：低侧 2-Shunt，ADC1/ADC2 双 ADC 注入同步采样  
> PWM：TIM1 中心对齐，20 kHz

---

## 1. 目标与硬件前提

电流采样结构：

```text
Ia -> ADC1 注入组 Rank1
Ib -> ADC2 注入组 Rank1
Ic = -(Ia + Ib)
```

PWM 结构：

```text
TIM1 CH1/CH2/CH3：三相互补 PWM
TIM1 CH4：仅作为 ADC 触发通道，不输出到 GPIO
PWM 频率：20 kHz
```

ADC1 与 ADC2采用双 ADC 注入同步模式，使两个相电流由同一个硬件事件开始采样。

---

## 2. PWM 与 ADC 触发链路

触发链路：

```text
TIM1_CH4 OC4REF
        ↓
TIM1_TRGO2
        ↓
ADC1/ADC2 注入组
        ↓
ADC1/ADC2 同时采样
```

HAL 配置要求：

```text
TIM1_CH4：
    OCMode = TIM_OCMODE_PWM2
    Pulse  = ADC_TRIGGER_CCR
    不配置 GPIO

TIM1 Master：
    MasterOutputTrigger2 = TIM_TRGO2_OC4REF

ADC1/ADC2 注入组：
    ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2
    ExternalTrigInjecConvEdge =
        ADC_EXTERNALTRIGINJECCONV_EDGE_RISING
```

不要使用：

```text
TIM_TRGO2_OC4REF_RISINGFALLING
```

否则 OC4REF 上升沿和下降沿都可能触发 ADC，导致一个 PWM 周期采样两次。

---

## 3. PWM 周期计算

定义：

```text
fTIM：TIM1 实际计数时钟
fPWM：PWM 频率
PSC ：TIM1 预分频
ARR ：TIM1 自动重装值
```

中心对齐 PWM：

```text
ARR = fTIM / [2 × fPWM × (PSC + 1)] - 1
```

示例：

```text
fTIM = 170 MHz
fPWM = 20 kHz
PSC  = 0
```

得到：

```text
ARR = 170000000 / (2 × 20000) - 1
    = 4249
```

必须使用 CubeMX Clock Configuration 中显示的 TIM1 Timer Clock，不能无条件使用 `SystemCoreClock`。

---

## 4. ADC 触发点设计

### 4.1 目标采样位置

低侧 2-Shunt 采样时，建议把 ADC 采样保持窗口放在中心对齐 PWM 的 ARR 附近：

```text
CNT 向上计数到 CCR4
        ↓
ADC 开始采样保持
        ↓
CNT 到达 ARR 附近
        ↓
ADC 采样保持结束
```

ARR 附近通常对应公共低侧导通零矢量区的中心，远离 PWM 比较边沿，适合进行相电流采样。

触发点：

```text
CCR4 = ARR - ADC_SAMPLE_TICKS
```

其中：

```text
ADC_SAMPLE_TIME =
    ADC_SAMPLE_CYCLES / fADC

ADC_SAMPLE_TICKS =
    ceil(ADC_SAMPLE_TIME × fTIM)
```

也就是：

```text
ADC_SAMPLE_TICKS =
    ceil(
        ADC_SAMPLE_CYCLES × fTIM / fADC
    )
```

### 4.2 不要把 ADC 转换时间算入提前量

12 位 ADC 的总转换时间为：

```text
总转换时间 =
    采样保持时间 + 12.5 个 ADC 时钟
```

触发提前量只需要按照采样保持时间计算。

后续 12.5 个转换周期只影响 JEOS 中断产生时间，不影响模拟信号被锁存的实际时刻。

### 4.3 数值示例

假设：

```text
fTIM = 170 MHz
fADC = 42.5 MHz
ADC SamplingTime = 12.5 cycles
```

采样保持时间：

```text
ADC_SAMPLE_TIME =
    12.5 / 42.5 MHz
    ≈ 294.1 ns
```

换算为 TIM1 计数：

```text
ADC_SAMPLE_TICKS =
    ceil(294.1 ns × 170 MHz)
    = 50
```

因此：

```text
CCR4 = 4249 - 50
     = 4199
```

初始参数：

```text
TIM1 ARR  = 4249
TIM1 CCR4 = 4199
```

---

## 5. 有效采样窗口判定

不能只设置：

```text
CCR4 = ARR - 50
```

还必须确认 ADC 开始采样之前，最后一次功率器件切换已经完成。

定义：

```text
CCR_MAX = max(CCR1, CCR2, CCR3)
```

`CCR_MAX` 表示向上计数阶段最后一个三相 PWM 开关边沿。

定义总消隐时间：

```text
T_BLANK =
    PWM 死区
  + 栅极驱动传播延迟
  + MOS 开关时间
  + 分流电阻尖峰或振铃时间
  + 电流运放建立时间
```

换算为 TIM1 计数：

```text
BLANK_TICKS =
    ceil(T_BLANK × fTIM)
```

必须满足：

```text
CCR_MAX + BLANK_TICKS <= CCR4
```

代入触发点公式：

```text
ARR - CCR_MAX
>=
BLANK_TICKS + ADC_SAMPLE_TICKS
```

这就是固定中心采样点的核心有效条件。

---

## 6. 固定触发点第一版方案

第一版代码采用固定触发点：

```text
CCR4 = ARR - ADC_SAMPLE_TICKS
```

同时限制最大调制度，使：

```text
CCR_MAX <= CCR4 - BLANK_TICKS
```

如果条件不满足，则当前 PWM 周期的采样窗口无效。

第一版软件建议直接采用以下处理：

1. 限制最大占空比或 SVPWM 调制度；
2. 保持上一周期电流值；
3. 设置采样无效标志；
4. 本周期不更新电流环积分项。

建议第一版优先限制调制度，保证每个 PWM 周期都有可靠的公共低侧导通窗口。

后续再扩展：

```text
动态移动采样点
扇区相关采样
切换采样相
PWM 边沿重构
```

---

## 7. ADC 配置要求

### 7.1 ADC1

```text
注入组 Rank1：Ia
InjectedNbrOfConversion = 1
外部触发：TIM1_TRGO2
触发边沿：Rising
AutoInjectedConv = DISABLE
InjectedDiscontinuousConvMode = DISABLE
QueueInjectedContext = DISABLE
Injected Oversampling = DISABLE
```

### 7.2 ADC2

```text
注入组 Rank1：Ib
其余配置与 ADC1 相同
```

两个 ADC 必须保持一致：

```text
相同 ADC 时钟
相同 SamplingTime
相同分辨率
相同触发源
相同触发边沿
```

双 ADC 模式：

```text
Mode = ADC_DUALMODE_INJECSIMULT
```

只使用 ADC1 主 ADC 的注入完成中断执行 FOC，避免 ADC1 和 ADC2 分别进入一次控制中断。

---

## 8. ADC 启动顺序

先执行 ADC 内部自校准：

```text
HAL_ADCEx_Calibration_Start(
    &hadc1,
    ADC_SINGLE_ENDED
)

HAL_ADCEx_Calibration_Start(
    &hadc2,
    ADC_SINGLE_ENDED
)
```

ADC 自校准必须在 ADC 尚未启动转换时执行。

ADC 内部自校准只能校准 ADC 本身，不能替代电流运放、偏置电路和外部采样链路的零点校准。

双 ADC 注入同步模式启动顺序：

```text
1. 先启动 ADC2 从机
2. 再启动 ADC1 主机，并使能注入完成中断
3. 最后启动 TIM1_CH4 触发
```

对应 HAL 调用顺序：

```text
HAL_ADCEx_InjectedStart(&hadc2)

HAL_ADCEx_InjectedStart_IT(&hadc1)

HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4)
```

---

## 9. 电流偏置校准状态机

偏置不能直接固定为 2048，应采集实际零电流时的 ADC 平均值。

状态机建议：

```text
CURRENT_CAL_IDLE
CURRENT_CAL_SETTLE
CURRENT_CAL_DISCARD
CURRENT_CAL_ACCUMULATE
CURRENT_CAL_CALCULATE
CURRENT_CAL_READY
CURRENT_CAL_ERROR
```

---

## 10. 偏置校准安全条件

进入校准前必须满足：

```text
电机停止
驱动器 EN 关闭
TIM1 CH1/CH2/CH3 主输出不启动
TIM1 CH1N/CH2N/CH3N 互补输出不启动
MOS 不发生开关动作
实际相电流为 0
```

TIM1 计数器和内部 CH4 触发仍然运行：

```text
TIM1_CH4 -> TRGO2 -> ADC1/ADC2
```

这样偏置校准和正常 FOC 使用完全相同的：

```text
ADC 通道
ADC 时钟
ADC 采样时间
ADC 触发点
ADC 注入组
运放配置
```

---

## 11. 模拟链路稳定等待

启动 ADC、OPAMP 和 TIM1_CH4 触发后，先等待：

```text
5 ms ～ 20 ms
```

建议默认：

```text
CURRENT_CAL_SETTLE_TIME_MS = 10 ms
```

等待时间作为可调参数。

---

## 12. 丢弃启动阶段样本

丢弃最开始：

```text
CURRENT_CAL_DISCARD_COUNT = 64
```

20 kHz 下对应：

```text
64 / 20000 = 3.2 ms
```

丢弃期间读取 ADC，但不参与累计。

---

## 13. 累计偏置样本

建议累计：

```text
CURRENT_CAL_SAMPLE_COUNT = 512
```

20 kHz 下对应：

```text
512 / 20000 = 25.6 ms
```

需要维护：

```text
uint64_t sum_a
uint64_t sum_b

uint16_t min_a
uint16_t max_a
uint16_t min_b
uint16_t max_b

uint16_t sample_count
```

ADC 注入完成中断中：

```text
sum_a += ADC1_JDR1
sum_b += ADC2_JDR1

更新 min/max

sample_count++
```

中断中不要执行：

```text
64 位除法
浮点运算
复杂判断
日志打印
```

---

## 14. 偏置计算

累计完成后，在主循环或状态机任务中计算：

```text
offset_a =
    (sum_a + SAMPLE_COUNT / 2)
    / SAMPLE_COUNT

offset_b =
    (sum_b + SAMPLE_COUNT / 2)
    / SAMPLE_COUNT
```

因为 512 是 2 的幂，也可以：

```text
offset_a = (sum_a + 256) >> 9
offset_b = (sum_b + 256) >> 9
```

加 256 用于四舍五入。

---

## 15. 偏置有效性检查

### 15.1 偏置范围

12 位 ADC：

```text
ADC 满量程 = 4095
```

不要要求偏置必须精确等于 2048。

通用初始允许范围：

```text
ADC 满量程的 20% ～ 80%
```

即：

```text
819 ～ 3276
```

如果硬件设计偏置为 1.65 V，可根据实测将范围收紧，例如：

```text
1500 ～ 2600
```

### 15.2 噪声跨度

计算：

```text
span_a = max_a - min_a
span_b = max_b - min_b
```

初始门限建议：

```text
CURRENT_CAL_MAX_SPAN = 30 ～ 50 LSB
```

最终根据实际板卡噪声调整。

跨度超限可能表示：

```text
电机仍有电流
功率级仍在开关
运放未稳定
ADC 触发点处于干扰区域
模拟电源不稳定
硬件采样链路异常
```

### 15.3 均值漂移

把 512 次采样分成前后各 256 次：

```text
offset_first_half
offset_second_half
```

要求：

```text
abs(offset_first_half - offset_second_half)
<= CURRENT_CAL_DRIFT_LIMIT
```

初始建议：

```text
CURRENT_CAL_DRIFT_LIMIT = 5 ～ 10 LSB
```

### 15.4 失败重试

校准失败时：

```text
保持驱动器关闭
重新等待稳定
重新执行校准
```

建议：

```text
CURRENT_CAL_MAX_RETRY = 3
```

连续失败后进入：

```text
CURRENT_CAL_ERROR
```

并禁止启动电机。

---

## 16. ADC 注入中断工作模式

ADC1 注入完成中断根据状态机分流。

### 16.1 校准阶段

```text
如果 state == CURRENT_CAL_DISCARD：
    丢弃样本

如果 state == CURRENT_CAL_ACCUMULATE：
    读取 ADC1/ADC2
    累加采样值
    更新 min/max
    更新前半段和后半段累计值
    达到 512 次后切换到 CURRENT_CAL_CALCULATE
```

### 16.2 正常 FOC 阶段

```text
如果 state == CURRENT_CAL_READY：
    读取 ADC1_JDR1
    读取 ADC2_JDR1

    ia_code = raw_a - offset_a
    ib_code = raw_b - offset_b
    ic_code = -(ia_code + ib_code)

    执行 Clarke
    执行 Park
    执行 Id/Iq 电流环
    执行反 Park
    执行 SVPWM
```

电流 ADC 码必须使用有符号类型：

```text
int32_t ia_code
int32_t ib_code
int32_t ic_code
```

不能用无符号类型直接做偏置相减，否则低于偏置时会发生下溢。

---

## 17. 偏置处理方式

第一版推荐软件扣偏置：

```text
ia_code = raw_a - offset_a
ib_code = raw_b - offset_b
```

暂时不使用 ADC 硬件 Offset 寄存器，原因：

```text
方便观察 ADC 原始值
方便观察零点漂移
方便故障诊断
避免硬件饱和配置导致负电流信息丢失
```

系统稳定后，再决定是否使用 ADC OFR 硬件偏置功能。

---

## 18. 完整启动流程

```text
系统上电
  ↓
初始化系统时钟
  ↓
初始化 GPIO
  ↓
初始化 OPAMP
  ↓
初始化 TIM1
  ↓
初始化 ADC1/ADC2
  ↓
ADC1、ADC2 内部自校准
  ↓
配置 ADC1/ADC2 双注入同步模式
  ↓
关闭栅极驱动 EN
  ↓
确保 CH1/CH2/CH3 和互补输出未启动
  ↓
先启动 ADC2 注入组
  ↓
再启动 ADC1 注入组及中断
  ↓
启动 TIM1_CH4 内部触发
  ↓
等待模拟链路稳定
  ↓
丢弃 64 次采样
  ↓
累计 512 次偏置采样
  ↓
计算偏置
  ↓
检查偏置范围、噪声跨度和均值漂移
  ↓
校准成功
  ↓
启动 TIM1 CH1/CH2/CH3 主输出
  ↓
启动 CH1N/CH2N/CH3N 互补输出
  ↓
最后使能栅极驱动器 EN
  ↓
进入 FOC
```

---

## 19. 建议的软件模块接口

### `CurrentSense_Init()`

负责：

```text
初始化采样模块变量
清除校准状态
设置偏置无效
初始化采样有效标志
```

### `CurrentSense_CalibrationStart()`

负责：

```text
关闭功率输出
关闭驱动器 EN
清空累计值
清空 min/max
清空前后半段累计值
进入 CURRENT_CAL_SETTLE
```

### `CurrentSense_InjectedCallback()`

由 ADC1 注入完成中断调用，负责：

```text
读取 ADC1/ADC2
校准阶段丢弃或累计数据
正常阶段生成有符号相电流
检查本周期采样窗口是否有效
调用 FOC 电流环
```

### `CurrentSense_Process()`

由主循环调用，负责：

```text
稳定等待计时
偏置平均值计算
偏置范围检查
噪声跨度检查
均值漂移检查
失败重试
状态切换
```

### `CurrentSense_IsReady()`

返回：

```text
偏置是否校准成功
是否允许开启驱动器
是否允许进入 FOC
```

### 调试接口

```text
CurrentSense_GetRaw()
CurrentSense_GetOffset()
CurrentSense_GetCurrentCode()
CurrentSense_GetCalState()
CurrentSense_GetSampleValid()
```

---

## 20. 建议集中定义的参数

```text
TIM1_CLOCK_HZ
ADC_CLOCK_HZ
PWM_FREQUENCY_HZ
PWM_PRESCALER
PWM_ARR

ADC_SAMPLE_CYCLES
ADC_SAMPLE_TICKS
ADC_TRIGGER_CCR

CURRENT_BLANK_TIME_NS
CURRENT_BLANK_TICKS

CURRENT_CAL_SETTLE_TIME_MS
CURRENT_CAL_DISCARD_COUNT
CURRENT_CAL_SAMPLE_COUNT
CURRENT_CAL_MAX_SPAN
CURRENT_CAL_OFFSET_MIN
CURRENT_CAL_OFFSET_MAX
CURRENT_CAL_DRIFT_LIMIT
CURRENT_CAL_MAX_RETRY
```

禁止将以下数值散落为魔法数字：

```text
ARR = 4249
CCR4 = 4199
丢弃 64 次
累计 512 次
偏置范围
噪声跨度
漂移门限
```

---

## 21. 代码生成要求

交给其他 AI 编写代码时，明确要求：

1. 使用 STM32CubeMX 生成的 HAL 工程结构；
2. 不修改 HAL 库源码；
3. TIM1 CH4 仅内部使用，不配置 GPIO；
4. ADC1/ADC2 使用双注入同步模式；
5. ADC2 先启动，ADC1 后启动；
6. 只使用 ADC1 注入完成中断执行控制；
7. 偏置校准使用非阻塞状态机；
8. 中断中禁止浮点运算、除法和日志打印；
9. 偏置校准完成前禁止开启驱动器；
10. 运行中禁止自动更新偏置；
11. 仅在电机停止、驱动关闭、确认零电流时允许重新校准；
12. 检查每周期有效采样窗口；
13. 采样窗口无效时必须限制调制度或跳过本周期积分更新；
14. 所有计数和时间参数使用宏或配置结构统一管理；
15. 所有 HAL 返回值必须检查。

---

## 22. 验收要求

代码完成后必须验证：

1. ADC 每个 PWM 周期只触发一次，即 20 kHz；
2. ADC1 和 ADC2 来自同一个 TIM1_TRGO2 触发；
3. TIM1 CH4 使用 PWM2；
4. ADC 只响应 OC4REF 上升沿；
5. `CCR4` 位于向上计数阶段并靠近 ARR；
6. ADC 采样前满足消隐时间；
7. 电机未运行时，扣除偏置后的 Ia、Ib 平均值接近 0；
8. 校准期间 `max-min` 不超过门限；
9. 前后半段偏置差不超过漂移门限；
10. 最大占空比时仍满足：

```text
ARR - CCR_MAX
>=
BLANK_TICKS + ADC_SAMPLE_TICKS
```

11. 不满足采样窗口时，软件能够限制调制度或标记采样无效；
12. 偏置校准完成前，驱动器 EN 始终关闭；
13. 电机运行中偏置值保持不变；
14. 示波器确认 ADC 完成中断发生在触发之后；
15. 调试日志能够输出：

```text
raw_a
raw_b
offset_a
offset_b
span_a
span_b
drift_a
drift_b
CCR1
CCR2
CCR3
CCR4
sample_valid
cal_state
```

---

## 23. 初始推荐参数

```text
TIM1 Clock                 = 170 MHz
PWM Frequency              = 20 kHz
TIM1 Prescaler             = 0
TIM1 ARR                    = 4249

ADC Clock                  = 42.5 MHz
ADC Sampling Time          = 12.5 cycles
ADC Sample Ticks           = 50
TIM1 CCR4                  = 4199

Calibration Settle Time    = 10 ms
Calibration Discard Count  = 64
Calibration Sample Count   = 512
Calibration Max Span       = 30 ～ 50 LSB
Calibration Drift Limit    = 5 ～ 10 LSB
Calibration Max Retry      = 3
```

这些参数仅作为第一版起点，最终必须根据实际 ADC 时钟、死区、驱动传播延迟、MOS 开关速度、运放带宽和示波器波形进行调整。
