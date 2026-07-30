# AGENTS.md

This file provides guidance when working with this repository.

## Project Overview

STM32F407VETx electronic-design competition car. The current hardware consists of:

- Two RS485 stepper motors for a differential-drive chassis
- One RS485 stepper motor for a ball-balancing frame
- Eight digital infrared line sensors
- Two independent UART vision links: the retained general receiver and the
  Question 2 ball-position receiver
- A JY61P three-axis IMU
- An SSD1306-compatible hardware-I2C OLED
- Two active-low task-selection buttons

The project was generated with STM32CubeMX 6.8.0 and is built with Keil MDK-ARM.
Open `MDK-ARM/gc.uvprojx` in Keil to build and flash over ST-Link.

Preprocessor defines: `USE_HAL_DRIVER;STM32F407xx`

## Active Architecture

1. `zhangdatou.c/.h` — Low-level address-based stepper protocol on USART1.
2. `HWT101.c/.h` — WIT 0x55-protocol yaw and angular-velocity receiver,
   retained under its original name and now used with JY61P on USART2.
3. `serial.c/.h` — Retained line-based vision receiver on UART5. It accepts
   `x_error,y_error\n` and `none\n`.
4. `BallVision.c/.h` — Question 2 single-value receiver on USART6. It accepts
   a signed decimal ball position such as `-2.5\n`, or `none\n`.
5. `DS.c/.h` — Board mapping and the application-facing hardware facade. It owns
   motor address/direction mapping, reads all infrared inputs, aggregates vision
   and IMU data, and exposes chassis/balance-frame commands.
6. `button.c/.h` — Debounced, non-blocking PB6/PB7 active-low buttons.
7. `i2c.c/.h`, `OLED.c/.h` — STM32F4 HAL I2C1 OLED driver on PB8/PB9.
8. `LineFollow.c/.h` — 8-sensor weighted-centroid PD line follower with curve
   slowdown and IMU unwrapped-yaw one-lap stopping.
9. `BalanceControl.c/.h` — Question 2 position PID driving USART1 motor address
   `0x03`. The current phase regulates the ball to center (`target=0`); the
   later `+5 cm -> -5 cm` sequence has reserved targets and a target setter.
10. `DS_task.c/.h` — Question selection and start state machine. Question 1
    runs line following; Question 2 runs center-return control. Both display
    elapsed time and diagnostics on the OLED.
11. `PID.c/.h` — Generic PID library reused by the balance-frame controller.
12. `main.c` — Initializes the active peripherals and repeatedly calls
    `DS_Run()` and `DS_Task_Run()`.

TIM2 provides a 1 ms tick through `DS_1msTickFromISR()`. UART callbacks dispatch
to the motor, vision, and IMU handlers; each handler checks its UART instance.

## Hardware Map

| Function | Mapping |
|---|---|
| Left chassis motor | USART1 address `0x01` |
| Right chassis motor | USART1 address `0x02`, inverted |
| Balance-frame motor | USART1 address `0x03` |
| IMU (JY61P) | USART2, 9600 |
| General vision (retained) | UART5, 9600 |
| Question 2 ball vision | USART6, PC6 TX / PC7 RX, 9600 |
| Infrared 1 through 8, left to right | PE11, PE10, PE9, PE8, PE7, PA6, PA11, PA7 |
| Infrared active level | Low |
| Button 1 / question select | PB6, active-low, internal pull-up |
| Button 2 / confirm-start-stop | PB7, active-low, internal pull-up |
| OLED hardware I2C1 | PB8 SCL, PB9 SDA, AF4 open-drain, 100 kHz |

Motor addresses, motor directions, infrared pins, and the infrared active level
are centralized in `Core/Inc/DS.h`. Button mapping is in `Core/Inc/button.h`;
OLED peripheral mapping is in `Core/Src/i2c.c` and the HAL MSP setup. Update
these files and `gc.ioc` whenever the wiring changes.

## Retained Optional Modules

`WS2812`, `screen`, and `encoder_f407` remain in the project but are not
initialized by the active startup path. Keep them only if they become useful
for status indication, debugging, or odometry.

PC6 and PC7 are active again as USART6 TX/RX for Question 2 vision; the deleted
servo stack is not restored. UART4 and USART3 remain generated as spare UART
resources but are not initialized by `main.c`.

## Removed Logistics-Robot Modules

The four-wheel mecanum/Z-axis layer and logistics task chain were removed:

- `GC_task`, `GC_Chassis_Control`, `Motor_Move`, `action`
- `servo`, `LobotServoController`, `huaner_servo`, `ServoMotorControl`
- `QRcode`, `laser`, `bluetooth`

Do not reintroduce these modules for the new two-wheel car. Reuse `PID`
algorithms through a control module built on top of DS instead.

## CubeMX Pattern

Preserve `/* USER CODE BEGIN/END */` sections when regenerating. `gc.ioc` remains
the peripheral and pin source of truth. Keep `AGENTS.md` and `CLAUDE.md` in sync.

See `DS_PORTING.md` for the migration record and hardware checks required before
on-car testing. See `LINE_FOLLOW_TUNING.md` for the Question 1 algorithm and
tuning sequence, and `BALANCE_CONTROL_TUNING.md` for Question 2.
