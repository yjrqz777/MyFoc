# STM32CubeMX + HAL 使用 TIM PWM + 环形小缓冲 DMA 驱动 WS2812 完整实现规格

## 1. 项目目标

使用 STM32CubeMX 生成外设初始化代码，基于 STM32 HAL 库实现 WS2812 驱动。

驱动架构为：

```text
TIM PWM
+ PWM通道CCx DMA请求
+ Circular DMA
+ 双半区小缓冲区
+ DMA Half Transfer回调
+ DMA Transfer Complete回调
```

实现目标：

* TIM输出约800kHz PWM。
* 每个WS2812数据位对应一个CCR值。
* DMA自动逐个将CCR值写入TIM比较寄存器。
* CPU不在每个WS2812数据位进入中断。
* CPU只在DMA发送完半个缓冲区时批量生成后续数据。
* DMA缓冲区大小固定，不随灯珠总数量明显增大。
* 支持异步发送。
* 支持任意数量的普通24位RGB WS2812。
* 正确处理帧尾Reset低电平。
* 正确处理DMA环形模式的停止时机。
* 正确处理首个PWM数据错位问题。

本规范以大多数STM32CubeMX生成的传统HAL接口为基础：

```text
HAL_TIM_PWM_Start_DMA()
HAL_TIM_PWM_Stop_DMA()
```

不要使用每1.25μs进入一次CPU的TIM更新中断方案。

---

# 2. WS2812数据格式

普通WS2812每颗灯有三个8位颜色通道：

```text
Green：8位
Red：8位
Blue：8位
```

因此一颗灯需要：

```text
8 + 8 + 8 = 24位
```

常见发送顺序为：

```text
G7～G0
R7～R0
B7～B0
```

即：

```text
GRB
MSB First
```

每个颜色通道取值范围：

```text
0～255
```

注意：

* 24位是WS2812通信数据。
* 它不是MCU对LED进行24次软件PWM。
* WS2812收到24位数据后，由灯珠内部的PWM模块控制RGB亮度。

如果使用的不是普通WS2812，而是RGBW、SK6812 RGBW等器件，需要将每颗灯的数据位数改为32位。

---

# 3. WS2812时序设计

WS2812常见通信速率约为：

```text
800kbit/s
```

因此一个数据位的总周期约为：

```text
1 / 800000 = 1.25us
```

Code 0和Code 1的区别不是总周期，而是高电平持续时间：

```text
Code 0：较短的高电平
Code 1：较长的高电平
```

PWM周期保持相同，只改变CCR。

定义：

```text
CCR_CODE_0
CCR_CODE_1
CCR_RESET
```

通常：

```text
CCR_RESET = 0
```

初始占空比可以按以下比例设计：

```text
CCR_CODE_0 ≈ PWM周期计数值的1/3
CCR_CODE_1 ≈ PWM周期计数值的2/3
```

最终值必须结合所用灯珠的数据手册和示波器波形确定。

不要直接复制其他工程中的固定CCR数值，因为CCR取决于实际TIM计数时钟。

---

# 4. TIM时钟计算

必须使用TIM实际内核时钟计算，不能直接假设TIM时钟等于CPU主频。

需要根据：

* STM32具体系列
* TIM所在APB总线
* APB分频
* 定时器时钟倍频规则
* CubeMX Clock Configuration

确定：

```text
TIM_KERNEL_CLK
```

然后计算：

```text
TIM_COUNTER_CLK = TIM_KERNEL_CLK / (PSC + 1)
PWM_FREQ = TIM_COUNTER_CLK / (ARR + 1)
```

目标：

```text
PWM_FREQ ≈ 800000Hz
```

推荐优先设置：

```text
PSC = 0
```

然后：

```text
ARR + 1 = TIM_KERNEL_CLK / 800000
```

如果无法整除，应选择最接近的ARR，并检查：

* 实际PWM周期是否满足灯珠时序。
* Code 0高电平是否满足要求。
* Code 1高电平是否满足要求。

例如TIM计数时钟为72MHz：

```text
周期计数值 = 72000000 / 800000 = 90
ARR = 89
```

CCR可以从类似以下数值开始验证：

```text
CCR_CODE_0 ≈ 30
CCR_CODE_1 ≈ 60
```

这些数值只适用于90个计数构成一个周期的情况。

---

# 5. CubeMX中的TIM配置

选择一个带DMA请求能力的TIM PWM通道，例如：

```text
TIMx
PWM Generation CH1
```

推荐TIM参数：

```text
Counter Mode           = Up
Prescaler              = 根据实际TIM时钟计算
Counter Period         = ARR
Clock Division         = DIV1
Repetition Counter     = 0
Auto-reload Preload    = Disable或Enable均可
```

PWM通道参数：

```text
PWM Mode               = PWM mode 1
Pulse                  = 0
Output Polarity        = High
Fast Mode              = Disable
```

GPIO参数：

```text
Mode                    = Alternate Function Push Pull
Pull                    = No Pull
Speed                   = High或Very High
```

不要开启：

```text
TIM Update Interrupt
TIM Global Interrupt
```

本方案不需要CPU处理每个PWM周期。

---

# 6. CCR Preload要求

使用PWM通道比较事件请求DMA时，DMA可能在当前PWM周期的比较点写入CCR。

如果写入的新CCR立即生效，可能破坏当前PWM周期，产生异常脉宽。

因此应确保PWM通道启用了CCR预装载：

```text
DMA写入CCR Preload
        ↓
当前PWM周期不受影响
        ↓
下一个Update Event
        ↓
Preload值装载到有效CCR
        ↓
下一周期使用新占空比
```

ST官方PWM DMA示例同样指出：使用比较事件更新PWM占空比时，应使用捕获比较预装载，使新的比较值在更新事件处同步生效，避免当前周期中途改变脉冲宽度。

其他AI实现时必须检查目标系列HAL初始化后：

```text
OCxPE
```

是否已经开启。

---

# 7. CubeMX中的DMA配置

在TIM的DMA Settings中添加对应PWM通道的DMA请求，例如：

```text
TIMx_CH1
```

如果使用的是：

```text
HAL_TIM_PWM_Start_DMA()
```

则默认应使用对应的：

```text
TIM DMA CC1
TIM DMA CC2
TIM DMA CC3
TIM DMA CC4
```

不要把本方案配置成：

```text
TIMx_UP
```

除非放弃标准`HAL_TIM_PWM_Start_DMA()`流程，改为自己使用底层HAL DMA接口和TIM UDE进行控制。

传统STM32 HAL中的PWM DMA启动接口属于PWM通道DMA流程；HAL驱动也提供PWM完成和PWM半完成回调。

DMA推荐参数：

```text
Direction                    = Memory to Peripheral
Mode                         = Circular
Peripheral Increment         = Disable
Memory Increment             = Enable
Priority                     = High或Very High
```

数据宽度根据具体芯片确定。

对于常见16位TIM：

```text
Peripheral Data Alignment    = Half Word
Memory Data Alignment        = Half Word
```

但其他AI必须检查：

* 目标TIM是16位还是32位。
* CCR寄存器访问宽度。
* 目标系列HAL对DMA数据指针和宽度的要求。
* CubeMX生成的DMA配置。

不要仅根据其他STM32系列的示例直接确定数据宽度。

必须开启对应DMA通道的NVIC中断。

---

# 8. CubeMX生成代码检查

生成代码后必须检查以下内容。

## 8.1 DMA句柄关联

确认TIM句柄对应的DMA数组正确关联到PWM通道，例如逻辑上应满足：

```text
PWM CH1 → TIM_DMA_ID_CC1
PWM CH2 → TIM_DMA_ID_CC2
PWM CH3 → TIM_DMA_ID_CC3
PWM CH4 → TIM_DMA_ID_CC4
```

CubeMX通常通过：

```text
__HAL_LINKDMA
```

完成关联。

如果关联错误，可能出现：

* `HAL_TIM_PWM_Start_DMA()`返回错误。
* DMA可以启动但没有回调。
* DMA写入错误的CCR。
* 空指针异常。
* 其他TIM通道被错误操作。

## 8.2 DMA中断函数

确认对应的DMA IRQ Handler中调用了：

```text
HAL_DMA_IRQHandler()
```

否则硬件会产生HT、TC标志，但HAL回调不会执行。

## 8.3 TIM中断

本方案不要求TIM Update IRQ。

需要的是：

```text
DMA IRQ
```

而不是：

```text
TIM IRQ
```

---

# 9. DMA小型环形缓冲区

DMA缓冲区划分为两个相等的半区：

```text
DMA Buffer
├── 前半区
└── 后半区
```

推荐每个半区容纳4颗LED。

一颗LED：

```text
24个CCR元素
```

每半区：

```text
4 × 24 = 96个CCR元素
```

整个缓冲区：

```text
2 × 96 = 192个CCR元素
```

如果每个元素为16位：

```text
192 × 2 = 384字节
```

定义关系：

```text
LEDS_PER_HALF = 4
BITS_PER_LED = 24

HALF_BUFFER_SLOTS =
LEDS_PER_HALF × BITS_PER_LED

FULL_BUFFER_SLOTS =
HALF_BUFFER_SLOTS × 2
```

推荐保证：

```text
HALF_BUFFER_SLOTS是24的整数倍
```

这样每个半区总是在完整LED边界结束，状态管理更简单。

---

# 10. 为什么不是每个Code 0、Code 1进入中断

DMA缓冲区中的每一个元素代表一个PWM周期的CCR。

例如WS2812数据是：

```text
1 0 1 1 0
```

DMA缓冲区存放：

```text
CCR_CODE_1
CCR_CODE_0
CCR_CODE_1
CCR_CODE_1
CCR_CODE_0
```

运行时：

```text
TIM产生PWM比较事件
        ↓
TIM向DMA发送请求
        ↓
DMA读取下一个缓冲区元素
        ↓
DMA写入TIM CCR
```

整个过程由TIM和DMA硬件自动完成。

CPU不会每1.25μs进入一次中断。

CPU只在以下时刻进入DMA中断：

```text
前半区发送完成 → HT
后半区发送完成 → TC
```

如果每半区保存4颗LED：

```text
4 × 24 × 1.25us = 120us
```

即大约每120μs进入一次DMA回调，而不是每1.25μs进入一次。

---

# 11. RGB显存设计

RGB显存只保存颜色数据，不保存展开后的PWM波形。

每颗灯可以逻辑上保存：

```text
R
G
B
```

发送时转换为：

```text
G
R
B
```

显存占用：

```text
LED_COUNT × 3字节
```

DMA工作缓冲区固定为小缓冲区，两者必须分开：

```text
RGB显存：
保存所有灯的颜色

PWM DMA缓冲区：
只保存当前正在发送的少量灯的CCR
```

对于灯效动画，建议使用双RGB显存：

```text
前台发送缓冲区
后台编辑缓冲区
```

调用Show时交换两个RGB缓冲区指针。

这样可以避免DMA发送过程中应用修改RGB数据，造成：

```text
前半条灯带使用旧帧
后半条灯带使用新帧
```

如果不使用双RGB显存，则发送Busy期间禁止修改当前发送显存。

---

# 12. 驱动状态机

建议至少包含以下状态：

```text
IDLE
DATA
RESET
DRAIN
STOPPING
ERROR
```

## IDLE

驱动空闲。

允许：

* 修改RGB显存。
* 调用Show启动发送。

## DATA

继续将RGB数据转换为CCR。

编码顺序：

```text
G7～G0
R7～R0
B7～B0
```

## RESET

所有LED数据已经编码完成。

后续DMA缓冲区填入：

```text
CCR_RESET = 0
```

用于产生帧尾低电平。

## DRAIN

所有需要的Reset Slot都已经写入DMA缓冲区，但最后一批数据可能尚未真正经过DMA和TIM输出。

此时不能立即停止DMA。

必须等待保存最后Reset数据的半区真正发送完成。

## STOPPING

停止PWM DMA，确保输出保持低电平。

## ERROR

DMA错误、状态机错误或参数错误。

停止所有输出并清除Busy。

---

# 13. 核心运行变量

驱动至少需要维护以下逻辑变量：

```text
当前状态
Busy标志
当前LED索引
LED总数量
RGB发送缓冲区指针
Reset剩余Slot数量
前半区状态
后半区状态
最后Reset数据所在半区
是否正在等待最终半区发送完成
错误标志
```

主循环和DMA中断共同访问的变量应：

* 使用适当的`volatile`语义。
* 避免主循环和中断同时写同一个状态。
* 必要时使用非常短的临界区。
* 禁止在临界区中执行颜色编码或大块内存操作。

---

# 14. 半区填充逻辑

需要设计一个通用的“填充指定半区”逻辑。

输入信息：

```text
目标半区
半区起始地址
半区Slot数量
当前状态
当前LED索引
Reset剩余数量
```

填充规则如下。

## 14.1 DATA阶段

只要当前LED索引小于LED总数：

1. 读取一颗LED的R、G、B。
2. 按G、R、B顺序处理。
3. 每个字节从bit7处理到bit0。
4. bit为0时写入`CCR_CODE_0`。
5. bit为1时写入`CCR_CODE_1`。
6. 每颗LED生成24个CCR。
7. LED索引加1。
8. 继续处理下一颗LED。

如果在半区中间完成了最后一颗LED：

* 状态切换到RESET。
* 当前半区剩余位置立即开始填充0。
* 数据位之间不能插入额外空隙。

## 14.2 RESET阶段

每写入一个：

```text
CCR_RESET
```

就消耗一个Reset Slot。

当Reset剩余数量减到0时：

* 当前半区剩余位置仍然继续填0。
* 记录当前半区为“最终Reset半区”。
* 状态切换到DRAIN。

继续填0不会造成问题，只会使Reset低电平更长。

---

# 15. Reset时间

Reset时间必须做成配置项。

不建议把Reset时间固定为50μs，因为不同WS2812兼容器件的锁存时间可能不同。

通用默认值建议：

```text
RESET_TIME_US = 300us
```

Reset Slot数量：

```text
RESET_SLOT_COUNT =
向上取整(
    RESET_TIME_US ÷ 实际PWM周期
)
```

当PWM周期为1.25μs时：

```text
300us ÷ 1.25us = 240个Slot
```

Reset通过连续输出：

```text
CCR = 0
```

实现。

不要采用以下错误方式：

```text
最后一个LED数据位结束
        ↓
立即HAL_TIM_PWM_Stop_DMA()
```

这样可能：

* 截断最后一个数据位。
* 没有满足Reset时间。
* 灯珠不锁存。
* 灯带出现随机闪烁。

---

# 16. HAL回调职责

传统STM32 HAL提供PWM DMA半完成和完成回调。HAL驱动中明确包含PWM完成回调与PWM半完成回调。

## 16.1 半传输回调

使用：

```text
HAL_TIM_PWM_PulseFinishedHalfCpltCallback()
```

它表示：

```text
DMA刚刚发送完前半区
DMA当前正在发送后半区
```

因此CPU可以安全修改：

```text
前半区
```

不能修改：

```text
后半区
```

处理流程：

```text
检查是否为目标TIM
检查是否为目标PWM通道
检查驱动是否Busy
检查是否处于DRAIN完成点
否则填充前半区
```

## 16.2 传输完成回调

使用：

```text
HAL_TIM_PWM_PulseFinishedCallback()
```

在Circular模式下，它表示：

```text
DMA刚刚发送完后半区
DMA已经回到前半区继续发送
```

因此CPU可以安全修改：

```text
后半区
```

不能修改：

```text
前半区
```

处理流程：

```text
检查是否为目标TIM
检查是否为目标PWM通道
检查驱动是否Busy
检查是否处于DRAIN完成点
否则填充后半区
```

## 16.3 注意

Circular DMA中的Transfer Complete回调不代表整条灯带发送完成。

它只代表：

```text
DMA缓冲区后半区发送完成
```

DMA随后会自动回到缓冲区开头。

---

# 17. 回调中的TIM和通道过滤

HAL PWM回调是全局弱回调。

如果工程中存在多个TIM PWM DMA，必须判断：

```text
回调中的TIM句柄
是否等于WS2812使用的TIM句柄
```

还要判断当前活动通道是否等于：

```text
WS2812使用的TIM_CHANNEL_x
```

否则其他TIM或其他PWM通道的DMA事件可能误触发WS2812状态机。

---

# 18. 正确的DRAIN停止逻辑

这是整个方案最容易出错的部分。

必须区分：

```text
数据已经写入DMA缓冲区
DMA已经读取该数据
TIM已经实际输出该PWM周期
```

三者不是同一时刻。

例如：

```text
HT回调发生
        ↓
CPU向前半区写入最后一批Reset
```

此时前半区刚刚发送完，DMA正在发送后半区。

刚写入前半区的最后Reset数据要等到：

```text
DMA发送完后半区
        ↓
DMA绕回前半区
        ↓
DMA再次发送完整个前半区
        ↓
下一次HT回调
```

才表示这批Reset数据已经被DMA消费。

因此停止逻辑应为：

1. 在某个半区填入最后需要的Reset数据。
2. 记录这个半区编号。
3. 进入DRAIN状态。
4. 等待同一个半区下一次产生“发送完成”回调。
5. 再执行停止。

具体关系：

```text
最后Reset写入前半区
→ 等待下一次HT回调
→ 停止

最后Reset写入后半区
→ 等待下一次TC回调
→ 停止
```

停止时，另一个半区也应保持全0。

这样即使DMA在CPU进入回调前已经开始发送另一个半区，也只会继续输出低电平。

---

# 19. 启动前的双半区预填充

在启动DMA之前必须预先填满：

```text
前半区
后半区
```

启动前流程：

1. 检查驱动状态必须为IDLE。
2. 检查RGB指针和LED数量。
3. 设置Busy。
4. 锁定当前RGB显存或交换双缓冲。
5. 当前LED索引清零。
6. Reset剩余数量初始化。
7. 最终半区标志清除。
8. 状态设置为DATA。
9. 停止可能残留的TIM PWM DMA。
10. 清除DMA状态标志。
11. 清除TIM状态标志。
12. TIM计数器清零。
13. CCR设置为安全初值。
14. 填充前半区。
15. 填充后半区。
16. 启动PWM DMA。

如果LED数量非常少，可能在启动之前填充两个半区时就已经进入RESET或DRAIN，这是合法情况，状态机必须支持。

---

# 20. HAL启动接口

使用标准接口：

```text
HAL_TIM_PWM_Start_DMA()
```

传入：

```text
TIM句柄
PWM通道
完整DMA缓冲区地址
完整DMA缓冲区元素数量
```

HAL会负责：

* 配置DMA源地址。
* 配置DMA目标为对应CCRx。
* 配置DMA内部完成和半完成处理。
* 开启对应PWM通道DMA请求。
* 开启PWM通道。
* 启动定时器。

传统HAL TIM驱动将PWM DMA作为标准启动模式，并暴露对应PWM完成、半完成回调。

启动返回值必须检查：

```text
HAL_OK
HAL_BUSY
HAL_ERROR
```

启动失败时：

* 清除Busy。
* 状态进入ERROR或IDLE。
* 输出保持低电平。

---

# 21. 首位错位与启动延迟

使用CCx DMA请求和CCR Preload时，DMA写入的数据通常用于后续PWM周期，不一定用于当前周期。

启动时可能出现：

* 第一个CCR未及时生效。
* 开头多一个低电平周期。
* 第一位被重复。
* 整帧数据偏移一位。
* 第一个DMA请求时机与预期不同。

推荐使用“低电平前导Slot”降低启动敏感性。

在正式24位数据前增加若干个：

```text
CCR_RESET = 0
```

例如：

```text
2～8个低电平Slot
```

这些Slot不作为帧尾Reset计数，只用于：

* 等待DMA和CCR Preload流水线稳定。
* 避免启动阶段异常高脉冲。
* 简化第一位生效顺序。

由于WS2812在正式数据之前保持低电平是安全的，前导低电平可以比1.25μs更长。

但要注意：

* 不能在一帧数据发送中间产生超过Reset阈值的低电平。
* 启动前导只能出现在第一颗灯数据之前。

其他AI仍需结合具体STM32参考手册确认：

* CC DMA请求发生时机。
* CCR Preload装载时机。
* `HAL_TIM_PWM_Start_DMA()`的启动顺序。
* 首次Update Event行为。

最终必须通过逻辑分析仪确认。

---

# 22. 推荐的编码序列

一次完整发送在逻辑上应为：

```text
前导低电平Slot
+
LED0的24位GRB数据
+
LED1的24位GRB数据
+
……
+
最后一颗LED的24位GRB数据
+
Reset低电平Slot
+
额外安全低电平
```

前导低电平建议：

```text
2～8个Slot
```

帧尾Reset建议：

```text
至少300us
```

---

# 23. 停止接口

最终半区真正发送完成后，调用：

```text
HAL_TIM_PWM_Stop_DMA()
```

停止后需要确保：

```text
CCR = 0
输出引脚保持低电平
TIM计数器回到已知状态
Busy = 0
State = IDLE
```

如果停止PWM后GPIO复用状态下的输出电平不确定，可以根据目标TIM行为选择：

* 保持PWM通道关闭且CCR为0。
* 将通道输出强制为Inactive。
* 必要时临时将GPIO改为普通推挽低电平。
* 下一次启动前恢复复用功能。

通常只要停止前CCR为0，并确认通道关闭后的空闲电平为低，就不必频繁切换GPIO模式。

---

# 24. 回调中禁止执行的操作

DMA HT和TC回调中只执行必要的实时操作。

允许：

```text
状态判断
索引更新
RGB位展开
CCR缓冲区填充
Reset计数
设置停止标志
```

禁止：

```text
printf
串口阻塞发送
HAL_Delay
动态内存分配
复杂浮点计算
灯效动画计算
Flash擦写
文件系统操作
等待信号量
长时间关闭中断
```

颜色、亮度、HSV转换、Gamma校正等计算应尽量在Show之前完成。

---

# 25. 中断实时性要求

每半区包含K颗灯时，CPU填充时间预算约为：

```text
K × 24 × 1.25us
```

例如：

```text
K = 1 → 约30us
K = 2 → 约60us
K = 4 → 约120us
K = 8 → 约240us
```

推荐默认：

```text
K = 4
```

如果系统存在较长的高优先级中断，可以调整为：

```text
K = 8
```

代价是DMA缓冲区RAM增加。

必须避免：

* 超过半区发送时间的高优先级ISR。
* 超过半区发送时间的全局关中断。
* 高优先级中断中执行阻塞通信。
* 发送期间执行长时间Flash擦除。

如果CPU未能及时填充已发送半区，DMA绕回后会重新发送旧内容，造成：

* 颜色错误。
* 后续LED数据错位。
* 闪烁。
* 整条灯带锁存异常。

---

# 26. 驱动接口要求

建议向应用层提供以下接口。

## 初始化

负责：

* 保存TIM句柄和通道。
* 计算或检查ARR、CCR0、CCR1。
* 初始化显存。
* 初始化状态机。
* 检查DMA配置。
* 保证输出初始为低。

## 设置单颗LED颜色

只修改RGB显存。

不直接操作DMA或TIM。

## 设置全部LED颜色

批量修改RGB显存。

## 清屏

将RGB显存全部清零。

需要调用Show后才实际输出。

## Show

异步启动一次发送。

返回状态：

```text
OK
BUSY
PARAM_ERROR
HAL_ERROR
```

Show不能阻塞等待整帧结束。

## IsBusy

查询驱动是否正在发送。

## Complete Callback

当以下条件全部满足后调用：

```text
所有LED数据已发送
Reset时间已满足
DMA已停止
输出已保持低电平
```

## Error Callback

DMA错误或状态机异常时调用。

---

# 27. 重复调用Show的策略

驱动Busy期间再次调用Show时，推荐选择以下一种策略。

## 简单策略

直接返回：

```text
BUSY
```

应用等待当前发送结束后重试。

## 双缓冲策略

应用继续更新后台RGB显存。

当前发送完成后：

* 交换前后台显存。
* 启动下一帧。

不建议在当前DMA发送过程中直接更改正在使用的RGB发送显存。

---

# 28. DMA错误处理

DMA错误最终应进入TIM或DMA错误处理路径。

发生错误时执行：

1. 禁止对应TIM DMA请求。
2. 停止DMA。
3. 停止PWM。
4. CCR清零。
5. 输出保持低电平。
6. Busy清零。
7. 状态设置为ERROR。
8. 保存错误原因。
9. 调用错误回调。

下一次重新发送前，应重新初始化必要状态。

---

# 29. 带Cache芯片的处理

对于STM32F7、H7及其他带D-Cache的型号，需要确认DMA一致性。

DMA缓冲区必须：

* 位于DMA可访问的RAM。
* 不能放在DMA不可访问的DTCM。
* 满足Cache Line对齐要求。
* 每次CPU修改一个半区后，对该半区执行D-Cache Clean。
* 或将DMA缓冲区放入非缓存区域。

否则可能出现：

```text
CPU已经更新DMA缓冲区
但DMA仍读取到Cache清理前的旧数据
```

这会表现为随机重复旧颜色或半区数据。

没有D-Cache的STM32一般不需要处理Cache一致性。

---

# 30. 电气层注意事项

软件正确不代表硬件一定可靠。

需要检查：

* WS2812供电电压。
* MCU数据电平是否满足灯珠输入高电平要求。
* MCU和灯带必须共地。
* 数据线是否过长。
* 电源是否有足够电流。
* 灯带首端是否有足够去耦。
* 数据线上是否需要串联小电阻。
* 3.3V MCU驱动5V灯带不稳定时是否需要电平转换。

这些问题可能表现为：

* 第一颗灯随机闪烁。
* 灯珠数量增加后错误。
* 高亮白色时复位。
* 示波器时序正确但灯带不稳定。

---

# 31. 验证项目

## 31.1 PWM基础波形

先不接灯带，用逻辑分析仪检查：

```text
PWM周期约为1.25us
Code 0高电平正确
Code 1高电平正确
低电平总周期正确
```

## 31.2 单颗LED数据

测试：

```text
全灭
纯红
纯绿
纯蓝
白色
```

检查GRB顺序。

理论发送字节：

```text
纯红：00 FF 00
纯绿：FF 00 00
纯蓝：00 00 FF
白色：FF FF FF
```

这里表示实际线路上的GRB顺序。

## 31.3 位顺序

确认：

```text
bit7先发送
bit0最后发送
```

## 31.4 缓冲区边界

假设每半区为4颗灯，重点测试LED数量：

```text
1
2
3
4
5
7
8
9
```

也就是：

```text
K - 1
K
K + 1
2K - 1
2K
2K + 1
```

用于发现HT、TC边界错误。

## 31.5 Reset

确认最后一个数据位之后：

```text
线路持续低电平至少配置的Reset时间
```

检查停止DMA时是否产生异常高脉冲。

## 31.6 第一位

重点放大第一帧开头，检查：

* 是否多一个错误高脉冲。
* 第一位是否丢失。
* 第一位是否重复。
* 整帧是否偏移一位。
* 前导低电平是否正常。

## 31.7 压力测试

发送WS2812期间同时运行：

```text
UART中断
ADC DMA
控制算法
RTOS任务
其他定时器
```

确认不会出现旧半区重复发送。

---

# 32. 推荐默认配置

```text
驱动框架：
STM32CubeMX + HAL

启动接口：
HAL_TIM_PWM_Start_DMA()

停止接口：
HAL_TIM_PWM_Stop_DMA()

DMA请求：
对应TIM PWM通道的CCx DMA请求

DMA模式：
Circular

DMA中断：
Half Transfer
Transfer Complete
Transfer Error

TIM更新CPU中断：
关闭

PWM频率：
约800kHz

颜色格式：
GRB

位顺序：
MSB First

每颗LED：
24个CCR

每半区LED数量：
4

每半区CCR数量：
96

完整DMA缓冲区：
192个CCR

CCR Reset：
0

前导低电平：
2～8个Slot

帧尾Reset：
默认300us

DMA优先级：
High或Very High

发送模式：
异步

RGB显存：
推荐支持双缓冲
```

---

# 33. 交给代码生成AI的最终要求

请根据具体STM32型号，使用STM32CubeMX生成的HAL工程，实现以上WS2812驱动。

必须满足：

1. 使用TIM PWM产生约800kHz波形。
2. 使用`HAL_TIM_PWM_Start_DMA()`启动。
3. CubeMX选择对应TIM PWM通道的CCx DMA请求。
4. DMA使用Circular模式。
5. DMA缓冲区分为两个半区。
6. 每半区默认容纳4颗LED，即96个CCR。
7. 半传输回调只填充前半区。
8. 传输完成回调只填充后半区。
9. CPU不处理每一个WS2812数据位中断。
10. RGB数据按GRB、MSB First转换。
11. 支持任意灯珠数量。
12. LED数据结束后继续输出Reset低电平。
13. 记录最后Reset数据所在半区。
14. 等该半区真正发送完成后再停止DMA。
15. 不能在刚填入最后Reset数据时立即停止。
16. 正确处理CCR Preload。
17. 正确处理第一位生效和DMA启动流水线。
18. 建议增加2～8个前导低电平Slot。
19. 检查CubeMX生成的DMA句柄链接。
20. 检查DMA IRQ中调用`HAL_DMA_IRQHandler()`。
21. 回调中检查目标TIM和PWM通道。
22. 回调中禁止printf、延时和阻塞操作。
23. 提供Busy状态和异步Show接口。
24. 提供发送完成回调和错误回调。
25. 带D-Cache的MCU必须处理DMA Cache一致性。
26. 所有关键时序必须使用逻辑分析仪验证。
27. 不要使用每周期TIM CPU中断模拟WS2812波形。
28. 不要使用整条灯带完全展开的大型CCR数组。
29. 不要在发送过程中修改当前RGB发送显存。
30. 代码必须包含清晰的状态机和半区所有权说明。
