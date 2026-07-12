# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository focus

MyFOC2 is an STM32G4 motor-control firmware project built as a Keil uVision/MDK-ARM project. The configured target is `MDK-ARM/g474app.uvprojx`, using the STM32G474RETx device definition and the Keil ARMCC toolchain. CubeMX configuration is retained in `g474app.ioc`; generated peripheral code is under `Core/`.

The root `README.md` documents the board assumptions and recent button/task changes. It also calls out an important hardware/configuration discrepancy: the README discusses an STM32G474RBT6 (128 KB Flash), while the current CubeMX/Keil project is configured for an STM32G474RETx/RET6 (512 KB Flash). Treat the project configuration and linker script as authoritative for builds, and verify the actual hardware before changing memory or pin assumptions.

## Build and verification commands

Run these from the repository root in PowerShell:

```powershell
# Build the Keil target from the command line
D:\App\Keil\Keil_v5\UV4\UV4.exe -b MDK-ARM\g474app.uvprojx
```

A successful build is expected to report `0 Error(s), 0 Warning(s)` and writes target output beneath `MDK-ARM\g474app\` (the project is configured to create an executable and HEX file). If Keil is installed elsewhere, use that installation's `UV4.exe` path.

There is no repository-level Makefile, CMake application target, lint configuration, or first-party host test runner. No single application test command is available. `Drivers/CMSIS/DSP/DSP_Lib_TestSuite/CMakeLists.txt` belongs to the vendored CMSIS-DSP test suite, not the MyFOC2 firmware target; do not treat it as the project's normal build or test entry point.

For hardware/runtime verification, flash and debug the generated target with the configured Keil/J-Link setup. Runtime diagnostics use SEGGER RTT (buffer 0 for text and buffer 1 named `JScope_f32`); `user_system.c` emits a CSV-like current/debug stream.

## Architecture

### Startup and generated MCU layer

- `Core/Src/main.c` is the application entry point. It calls `HAL_Init()`, configures the clock, initializes GPIO/DMA/SPI/UART/ADC/TIM peripherals, initializes RTT and motor state, starts TIM2 PWM and the motor stack, and enables TIM7 interrupts.
- `Core/Src/*.c` and `Core/Inc/*.h` contain STM32 HAL/CubeMX-generated peripheral setup and MSP initialization. `Core/Src/stm32g4xx_it.c` dispatches NVIC interrupts to HAL handlers; keep custom behavior in user-code regions or application/BSP modules when possible.
- `g474app.ioc` is the CubeMX source configuration. Regenerating code can overwrite generated files, so review the diff and preserve project-specific behavior after regeneration.

### Application layer: `Code/UserApp`

- `user_motor.c` owns motor lifecycle and the fast control loop. The current implementation uses a ramped open-loop electrical angle and an Iq reference ramp, then calls the FOC current loop and writes phase voltage to the PWM BSP. The fast loop is marked `USER_MOTOR_FAST_CODE` and is invoked from the ADC injected-conversion completion callback.
- `user_foc.c` implements the current-loop math: Clarke transform, Park transform, d/q PI controllers with anti-windup, d/q voltage limiting, inverse transforms, and FOC state/reference accessors. Its nominal control period is 50 us (20 kHz).
- `user_system.c`, `user_display.c`, `user_button.c`, `user_button_fun.c`, and `user_time.c` implement cooperative foreground tasks, telemetry/display, button event handling, and time-related behavior.

### Board-support layer: `Code/UserBsp`

BSP modules wrap board peripherals and electrical conversion details:

- `bsp_adc.c`: ADC1 injected four-rank sampling (Ia/Ib/Ic/Ibus), startup calibration/offset accumulation, current conversion, and ADC2 polled measurements (including the potentiometer).
- `bsp_pwm.c`: TIM1 three-phase complementary PWM start/stop and conversion from signed voltage values to bounded CCR values. TIM1 dead-time/center-aligned details come from the CubeMX timer configuration.
- `bsp_hall.c`, `bsp_button.c`, `bsp_lcd.c`, `st7789v/`, and `ws2812/`: Hall inputs, buttons, display, and RGB LED support.
- `RTT/`: SEGGER RTT transport used for debug output and J-Scope data.

`Code/UserDrv` is currently a reserved lower-level driver area. Shared declarations and section attributes are in `Code/user_global.h`; `USER_MOTOR_CCMRAM` places data in `.ccmram`, and `USER_MOTOR_FAST_CODE` places routines in `.fastcode`.

### Timing and control flow

There are two distinct execution paths:

1. **Fast motor path:** ADC1 injected conversions are triggered by TIM1 (`ADC1` injected trigger is `ADC_EXTERNALTRIGINJEC_T1_CC4`). `BspAdc_UpdateInjected()` reads the four ranks and accumulates zero-current offsets for 1024 samples. Once sampling is ready, `HAL_ADCEx_InjectedConvCpltCallback()` calls `UserMotor_FastLoop()`, which performs current sampling, FOC, and PWM update. Keep this path short and ISR-safe; avoid blocking, slow peripheral polling, or RTT printing in it.
2. **Foreground task path:** `main()` repeatedly registers `PtTaskDisplay`, `PtTaskButton`, `PtTaskSystem`, and `PtTaskTime` through the macros in `Code/Task.h`. TIM7's period callback calls `TASK_TICK_UPDATE()` to decrement task delays. These protothread-style tasks resume from macro-generated switch/case states and should use `PT_WAIT_UNTIL()` rather than blocking delays.

TIM1 is the motor PWM/control timer; TIM2 is also started for the configured RGB/auxiliary PWM; TIM7 supplies the task scheduler tick. ADC1/2, DMA, SPI1/SPI3, and USART3 are configured in the generated layer and consumed by the BSP/application layers.

## Memory and project-file considerations

- `MDK-ARM/g474app.sct` maps 512 KB Flash at `0x08000000`, 32 KB CCM SRAM at `0x10000000` for `.ccmram*`/`.fastcode*`, and 96 KB SRAM at `0x20000000` for normal read-write/zero-init data.
- The Keil project uses ARMCC 5.06 update 7, defines `USE_HAL_DRIVER,STM32G474xx,__TARGET_FPU_VFP`, and includes `Core/Inc`, STM32 HAL/CMSIS headers, and the `Code` application/BSP paths.
- `MDK-ARM/g474app.uvprojx`, `MDK-ARM/g474app.sct`, `g474app.ioc`, generated `Core/` files, and the project output configuration must remain consistent when changing device, memory, or peripheral setup.
- Avoid editing vendored `Drivers/` code unless the task specifically targets the dependency. Prefer project code and CubeMX user-code regions for firmware changes.

## Change navigation

For a motor-control change, start with `Code/UserApp/user_motor.c` and `user_foc.c`, then inspect `bsp_adc.c`, `bsp_pwm.c`, `Core/Src/adc.c`, `Core/Src/tim.c`, and the interrupt dispatch path. For a task/UI/input change, start with `Code/Task.h` and the relevant `Code/UserApp` module, then inspect its BSP and the TIM7 callback in `Core/Src/main.c`. For pin/peripheral changes, update `g474app.ioc`/CubeMX and compare regenerated `Core/` files with the existing user-code regions before building.

## Local conventions and validation

- Match nearby C style: application code generally uses four-space indentation, module-prefixed Pascal-style APIs such as `UserButton_GetPressedMask` and `BspButton_ReadLevel`, lowercase layer-based file names, fixed-width integer types, and explicit unsigned suffixes such as `0u`. Avoid unrelated reformatting, especially in generated or imported code.
- For changes involving timing, GPIO, PWM, ADC, Hall sensors, or protothread scheduling, a successful build is only the minimum check; validate on hardware with the existing Keil/J-Link setup. Check RTT output for button, display, and telemetry changes.
- When adding a foreground task, register it in `Core/Src/main.c` and confirm its delay is compatible with the TIM7 tick. Do not claim a single-test command: the repository has no first-party host test runner.
- `AGENTS.md` is also repository guidance; preserve its more specific instructions on style, hardware validation, generated code, transient Keil artifacts, and release `.hex`/`.map` handling.
- Current generated build records should be treated as evidence rather than as a guarantee: a checked-in build log may contain `0 Error(s), 1 Warning(s)` for an unreachable statement in `Code/UserApp/user_motor.c`, while the README records an older warning-free build. Re-run the Keil command after changes and report the actual result.
