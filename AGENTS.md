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
4. `BallVision.c/.h` — Full-duplex Question 2 link on USART6. After `c2`
   start it accepts `B,error_px,velocity_px_s\n`, or `none\n`; after each
   balance-motor command it returns a non-blocking `F,...\n` control-feedback
   frame to MaixCAM, and `ok` stops the stream.
5. `DS.c/.h` — Board mapping and the application-facing hardware facade. It owns
   motor address/direction mapping, reads all infrared inputs, aggregates vision
   and IMU data, and exposes chassis/balance-frame commands.
6. `button.c/.h` — Debounced, non-blocking PB6/PB7 active-low buttons.
7. `i2c.c/.h`, `OLED.c/.h` — STM32F4 HAL I2C1 OLED driver on PB8/PB9.
8. `LineFollow.c/.h` — 8-sensor weighted-centroid PD line follower with curve
   slowdown and IMU unwrapped-yaw one-lap stopping.
9. `BalanceControl.c/.h` — Question 2 cascaded controller for USART1 motor
   address `0x03`. A dt-aware outer PID converts MaixCAM ball position and
   filtered velocity into a target rod angle; an inner proportional loop uses
   motor-position feedback to produce a bounded, slew-limited speed command.
   The signed motor-to-rod fit is `-0.36450137 rod deg / motor deg`, derived
   from the robust `a010e37` dataset; the horizontal motor angle is still
   captured at Question 2 start because that dataset's absolute zero was
   relative to one capture session. The fitted common target-angle ceiling is
   `6.0` degrees and the protection limit is `6.5` degrees; first-loop tuning
   currently restricts the target to `1.5` degrees, motor speed to `30`, slew
   to `8`, and disables both outer integral gains.
   The current phase regulates to center (`target=0`); the later
   `+5 cm -> -5 cm` sequence has reserved pixel targets and a target setter.
10. `MotorPositionMonitor.c/.h` — Shared non-blocking `0x36` position reader
    for Questions 2 and 9. Each task selects its own period; Question 9 uses
    20 ms for 50 Hz calibration sampling without changing Question 2.
11. `Question9Telemetry.c/.h` — Isolated 50 Hz non-blocking Question 9
    telemetry sender on USART6. It emits `Q9,...\n` frames containing motor
    position, zero-relative three-axis angles, validity and motion status.
12. `DS_task.c/.h` — Question selection and start state machine. Question 1
    runs line following, Question 2 runs center-return control, and Question 9
    displays motor 3 position while moving between independently adjustable
    upper/lower endpoints, without starting either controller or changing
    Question 2 settings.
13. `PID.c/.h` — Retained generic PID library. Question 2 uses its own
    dt-aware, gain-scheduled cascaded law in `BalanceControl`.
14. `main.c` — Initializes the active peripherals and repeatedly calls
    `DS_Run()` and `DS_Task_Run()`.

TIM2 provides a 1 ms tick through `DS_1msTickFromISR()`. UART RX callbacks
dispatch to the motor, vision, and IMU handlers; the UART TX callback releases
the active Question 2 feedback or Question 9 telemetry buffer. Each handler
checks its UART instance.

## Balance-Motor Startup Invariant

Immediately after `DS_Init()` and before `DS_Task_Init()`, `main.c` must command
balance motor address `0x03` to move `+240` relative pulses at speed `50` and
acceleration `10`. This is an intentional power-on pre-positioning move that
places the mechanism near its usable middle position. It is not a homing move
or a calibrated zero. Do not remove, invert or retune this command unless the
mechanical startup requirement explicitly changes. Question 9 collects motion,
motor-position and IMU telemetry after this pre-positioning, but it must not
write Question 2 zero, linkage-ratio, direction or controller parameters.
Question 2 calibration values may be changed only from a reviewed offline fit;
the current signed ratio and direction settings come from commit `a010e37`.

## Question 9 Data-Collection Invariant

Question 9 is a repeatable excitation and telemetry task, not an automatic
calibrator. `DS_TASK_Q9_UPPER_PULSES` and `DS_TASK_Q9_LOWER_PULSES` are
independently adjustable non-negative endpoint offsets relative to the
stable Question 9 start position; the current upper/lower values are
`210`/`150`. After PB7 confirmation, the position monitor must first observe
the power-on `+240` move and residual motion settle. Motor `0x03` then moves
first to the upper endpoint and continuously crosses between the upper and
lower endpoints at speed `50` and acceleration `20`. Do not reverse on a fixed
timer: require observed
position change followed by consecutive stable `0x36` samples, then keep the
configured endpoint dwell. The crossing command uses the sum of both offsets
so unequal amplitudes cannot accumulate drift. `Question9Telemetry` must keep
sending the existing 13-field `Q9,...\n` frames at 50 Hz on USART6 while the
motor-position monitor also runs at 50 Hz. Keep roughly 300 ms of endpoint
stability confirmation (`15` successful position updates at 20 ms). Do not
remove this communication, collapse the two endpoint settings into one,
replace the loop with a finite auto-calibration sequence, or let Question 9
write any `balance_control_config` field. Captured data is uploaded and
analyzed offline before reviewed results are manually entered into Question 2.

## Hardware Map

| Function | Mapping |
|---|---|
| Left chassis motor | USART1 address `0x01` |
| Right chassis motor | USART1 address `0x02`, inverted |
| Balance-frame motor | USART1 address `0x03` |
| IMU (JY61P) | USART2, 9600 |
| General vision (retained) | UART5, 9600 |
| Question 2 / Question 9 vision link | USART6, PC6 TX / PC7 RX, 115200 |
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

PC6 and PC7 are active again as USART6 TX/RX for Question 2 vision and
Question 9 outbound telemetry; the deleted servo stack is not restored. UART4
and USART3 remain generated as spare UART resources but are not initialized by
`main.c`.

## Removed Logistics-Robot Modules

The four-wheel mecanum/Z-axis layer and logistics task chain were removed:

- `GC_task`, `GC_Chassis_Control`, `Motor_Move`, `action`
- `servo`, `LobotServoController`, `huaner_servo`, `ServoMotorControl`
- `QRcode`, `laser`, `bluetooth`

Do not reintroduce these modules for the new two-wheel car. Build any new
control algorithms through a dedicated module on top of DS instead.

## CubeMX Pattern

Preserve `/* USER CODE BEGIN/END */` sections when regenerating. `gc.ioc` remains
the peripheral and pin source of truth. Keep `AGENTS.md` and `CLAUDE.md` in sync.

See `DS_PORTING.md` for the migration record and hardware checks required before
on-car testing. See `LINE_FOLLOW_TUNING.md` for the Question 1 algorithm and
tuning sequence, and `BALANCE_CONTROL_TUNING.md` for Question 2.
