# STM32G474RBT6 参数
[stm32g474cb.pdf](https://www.st.com.cn/resource/en/datasheet/stm32g474cb.pdf)
| 项目 | 参数 |
| --- | --- |
| 内核 | Arm Cortex-M4，带 FPU / DSP / MPU |
| 主频 | 最高 170 MHz |
| Flash | 128 KB |
| SRAM | 128 KB，包括 80 KB SRAM、16 KB SRAM、32 KB CCM SRAM |
| 封装 | LQFP64，10 x 10 mm |
| 引脚数 | 64 pins |
| 工作电压 | 1.71 V ~ 3.6 V |
| 温度等级 | -40 ~ +85 °C |
| GPIO | 约 52 个 I/O，具体以封装复用为准 |
| ADC | 5 个 12-bit ADC，最高 4 Msps |
| DAC | 7 路 12-bit DAC |
| 运放 / 比较器 | 6 个运放，7 个比较器 |
| 定时器 | HRTIM、3 个高级控制定时器、32-bit/16-bit 通用定时器等 |
| 通信接口 | I2C、SPI、USART/UART、LPUART、FDCAN、USB FS、UCPD、SAI |
| 硬件加速 | CORDIC、FMAC、RNG、CRC |
| 调试接口 | SWD / JTAG |

> 注意：`STM32G474RBT6` 为 128 KB Flash；当前工程说明中的 `STM32G474RETx` 为 512 KB Flash，迁移时需要同步检查 CubeMX 与 Keil 目标器件配置。

# MyFOC2 变更记录

## 2026-07-07

### 目录分层

当前用户代码按三层整理：

- `Code/UserApp`：应用层逻辑，例如电机控制、FOC 调用、显示任务、按键任务。
- `Code/UserBsp`：板级支持层，封装 ADC、PWM、Hall、LCD、Button 等板上资源。
- `Code/UserDrv`：预留底层驱动层，用于后续放更底层 IO/芯片驱动。

### 按键移植

完成 4 个按键接入：

```c
#define KEY1_Pin GPIO_PIN_6
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_7
#define KEY2_GPIO_Port GPIOC
#define KEY3_Pin GPIO_PIN_8
#define KEY3_GPIO_Port GPIOC
#define KEY4_Pin GPIO_PIN_9
#define KEY4_GPIO_Port GPIOC
```

相关文件：

- `Code/UserBsp/bsp_button.c`
- `Code/UserBsp/bsp_button.h`
- `Code/UserApp/user_button.c`
- `Code/UserApp/user_button.h`

BSP 层新增：

- `BspButton_ReadLevel(uint8_t button_id)`：读取 KEY1-KEY4 GPIO 电平。
- `BspButton_GetRawMask(void)`：返回 4 个按键原始电平掩码。

UserApp 层新增：

- 初始化 4 个 `Button` 对象。
- 通过 `PtTaskButton()` 每 5 ms 调用一次 `button_ticks()`。
- 支持按下、释放、单击、双击、长按、重复按下等事件。
- 通过 RTT 打印按键事件。

当前按键默认按下有效电平：

```c
#define USER_BUTTON_ACTIVE_LEVEL 0u
```

如果硬件实际是按下为高电平，需要改为：

```c
#define USER_BUTTON_ACTIVE_LEVEL 1u
```

### PT 协程调度

`main.c` 中保留 PT 协程框架，并新增按键任务注册：

```c
PT_TASK_REG(0, PtTaskDisplay);
PT_TASK_REG(1, PtTaskButton);
```

TIM7 周期中断继续调用：

```c
TASK_TICK_UPDATE();
```

### 屏幕显示

`Code/UserApp/user_display.c` 中增加按键状态显示：

- `x=96, y=24`：4 个按键按下状态掩码。
- `x=96, y=48/72/96/120`：KEY1-KEY4 当前是否按下。
- `x=144, y=48/72/96/120`：KEY1-KEY4 最后一次事件编号。

原有 Hall 和 ADC 显示保留。

### Keil 工程更新

已将以下文件加入 Keil 工程：

- `Code/UserApp/user_button.c`
- `Code/UserBsp/bsp_button.c`

更新文件：

- `MDK-ARM/g474app.uvprojx`
- `MDK-ARM/g474app.uvoptx`

### 构建验证

使用 Keil 命令行构建通过：

```powershell
D:\App\Keil\Keil_v5\UV4\UV4.exe -b MDK-ARM\g474app.uvprojx
```

结果：

```text
0 Error(s), 0 Warning(s)
```
