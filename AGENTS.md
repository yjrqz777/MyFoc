# Repository Guidelines

## Project Structure & Module Organization

This is an STM32G474RETx firmware project generated from CubeMX and built with Keil uVision. CubeMX-managed HAL entry points live in `Core/Inc` and `Core/Src`; keep user edits inside the generated `USER CODE` blocks when modifying these files. Custom firmware is under `Code`: `Code/UserApp` contains application logic such as FOC, motor, display, and button tasks; `Code/UserBsp` wraps board peripherals such as ADC, PWM, Hall, LCD, buttons, ST7789V, and SEGGER RTT; `Code/UserDrv` is reserved for lower-level device drivers. Vendor code is under `Drivers`. Keil project files and build outputs are under `MDK-ARM`, with the active target output in `MDK-ARM/g474app`.

## Build, Test, and Development Commands

- `D:\App\Keil\Keil_v5\UV4\UV4.exe -b MDK-ARM\g474app.uvprojx`: batch-builds the `g474app` target and emits `.axf`, `.hex`, `.map`, and build logs.
- Open `g474app.ioc` in STM32CubeMX to change pin, clock, DMA, or peripheral configuration, then regenerate code and review `Core` changes carefully.
- Use Keil uVision for flashing/debugging with the existing `MDK-ARM/DebugConfig` and J-Link settings.

There is no standalone host test runner in this repository. Treat a clean Keil build with `0 Error(s)` as the minimum validation step.

## Coding Style & Naming Conventions

Use C99-compatible C. Match nearby formatting: four-space indentation is common in application files, while some imported button code uses tabs; avoid reformatting unrelated vendor or generated code. Public APIs use module prefixes and Pascal-style function names, for example `UserButton_GetPressedMask` and `BspButton_ReadLevel`. File pairs should stay lowercase by layer, such as `user_display.c/.h` or `bsp_pwm.c/.h`. Use `uint8_t`, `uint16_t`, and explicit suffixes like `0u` for fixed-width embedded values.

## Testing Guidelines

For changes that touch timing, GPIO, PWM, ADC, Hall sensors, or protothread scheduling, verify on hardware after building. Check RTT output when modifying `Code/UserApp/user_button.c` or other debug-logged paths. When adding tasks, confirm registration in `Core/Src/main.c` and make sure `TASK_TICK_UPDATE()` timing remains compatible with the task interval.

## Commit & Pull Request Guidelines

Recent history uses short subjects, sometimes conventional prefixes such as `refactor:`. Prefer imperative, scoped messages like `fix: debounce key release` or `refactor: split motor BSP`. PRs should describe the hardware behavior changed, list build results, mention CubeMX regeneration if used, and include screenshots or RTT/log snippets for UI/display or input behavior. Avoid committing transient Keil intermediates such as `.o`, `.d`, `.crf`, and `.__i`; commit `.hex` or `.map` only when intentionally updating release artifacts.

## Agent-Specific Instructions

Do not rewrite generated HAL or vendor driver files unless the task requires it. Keep edits focused in `Code/UserApp`, `Code/UserBsp`, project configuration, or documented `USER CODE` sections, and preserve existing Chinese/English comments unless updating the related behavior.
