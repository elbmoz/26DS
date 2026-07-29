# AGENTS.md

This file provides guidance when working with this repository.

## Project Overview

STM32F407VETx electronic-design competition car. The current hardware consists of:

- Two RS485 stepper motors for a differential-drive chassis
- One RS485 stepper motor for a ball-balancing frame
- Eight digital infrared line sensors
- A UART vision module
- An HWT101 IMU

The project was generated with STM32CubeMX 6.8.0 and is built with Keil MDK-ARM.
Open `MDK-ARM/gc.uvprojx` in Keil to build and flash over ST-Link.

Preprocessor defines: `USE_HAL_DRIVER;STM32F407xx`

## Active Architecture

1. `zhangdatou.c/.h` — Low-level address-based stepper protocol on USART1.
2. `HWT101.c/.h` — HWT101 yaw and angular-velocity receiver on USART2.
3. `serial.c/.h` — Line-based vision receiver on UART5. It accepts
   `x_error,y_error\n` and `none\n`.
4. `DS.c/.h` — The board mapping and application-facing hardware facade.
   It owns the motor address/direction mapping, reads all eight infrared inputs,
   aggregates vision and IMU data, and exposes chassis/balance-frame commands.
5. `PID.c/.h` — Generic PID library retained for future line, heading, and
   balance-frame control loops.
6. `main.c` — Initializes UART5, USART1, USART2, TIM2, and DS, then repeatedly
   calls `DS_Run()`.

TIM2 provides a 1 ms tick through `DS_1msTickFromISR()`. UART callbacks dispatch
to the motor, vision, and HWT101 handlers; each handler checks its UART instance.

## Hardware Map

| Function | Mapping |
|---|---|
| Left chassis motor | USART1 address `0x01` |
| Right chassis motor | USART1 address `0x02`, inverted |
| Balance-frame motor | USART1 address `0x03` |
| IMU | USART2, 115200 |
| Vision | UART5, 9600 |
| Infrared 1 through 8, left to right | PE11, PE10, PE9, PE8, PE7, PA6, PA11, PA7 |
| Infrared active level | Low |

All addresses, directions, pins, and active levels are centralized in
`Core/Inc/DS.h`. Update that file and `gc.ioc` whenever the wiring changes.

## Retained Optional Modules

`button`, `WS2812`, `screen`, and `encoder_f407` remain in the project but are
not initialized by the active startup path. Keep them only if they become useful
for start control, status indication, debugging, or odometry.

The old `button` mapping also uses PA6, which now belongs to infrared sensor 6.
Do not call `Button_Init()` until the button has been assigned a different pin.

UART4, USART3, and USART6 are still generated as spare UART resources but are not
initialized by `main.c`.

## Removed Logistics-Robot Modules

The four-wheel mecanum/Z-axis layer and logistics task chain were removed:

- `GC_task`, `GC_Chassis_Control`, `Motor_Move`, `action`
- `servo`, `LobotServoController`, `huaner_servo`, `ServoMotorControl`
- `QRcode`, `laser`, `bluetooth`

Do not reintroduce these modules for the new two-wheel car. Reuse `PID` algorithms
through a new control module built on top of DS instead.

## CubeMX Pattern

Preserve `/* USER CODE BEGIN/END */` sections when regenerating. `gc.ioc` remains
the peripheral and pin source of truth. Keep `AGENTS.md` and `CLAUDE.md` in sync.

See `DS_PORTING.md` for the migration record and hardware checks required before
on-car testing.
