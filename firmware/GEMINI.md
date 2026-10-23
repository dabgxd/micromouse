# firmware_micromouse — Workspace Rules

## Project Context
This is a **U-shaped maze obstacle-avoidance robot** firmware for the STM32F411CEU6 (WeAct Black Pill V2.0), built with STM32CubeMX + PlatformIO + STM32Cube HAL framework.

## Mandatory Skill
Always consult `.agents/skills/u-maze-robot/SKILL.md` before making any hardware-related, pin-assignment, peripheral-config, or algorithm changes.

## Code Style
- All user code MUST go inside `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks to survive CubeMX re-generation.
- Use descriptive `#define` labels for all GPIO pins (e.g., `AIN1_Pin`, `XSHUT_LEFT_Pin`) in `config.h`.
- C99 with `stdint.h` types. No dynamic allocation (`malloc`).
- Comments in English; variable names in English.

## Hardware Safety
- **NEVER** enable USB or connect 3.3V from ST-Link while battery power is active.
- PA13/PA14 must always remain as SWD — never reconfigure.
- TB6612FNG STBY (PA10) must be set HIGH before any motor PWM output.

