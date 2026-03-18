# Four-Wheel Drive Electric Vehicle — Central Control Unit

Firmware for an STM32-based central controller (CCU) of a four-wheel-drive electric vehicle (4WD e-vehicle). The CCU runs on a **NUCLEO-F446RE** development board and orchestrates all drive, steering, and power subsystems over a **CAN bus**.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [System Architecture](#system-architecture)
- [CAN Bus Protocol](#can-bus-protocol)
- [Startup State Machine](#startup-state-machine)
- [Drive Control — Ackermann Steering](#drive-control--ackermann-steering)
- [UART Command Interface](#uart-command-interface)
- [Peripherals & Pin Mapping](#peripherals--pin-mapping)
- [Project Structure](#project-structure)
- [Building & Flashing](#building--flashing)
- [Configuration](#configuration)

---

## Overview

This firmware acts as the **master controller** of a 4WD electric vehicle. It is responsible for:

- **Coordinating four independent wheel motor drivers** via CAN bus (front-right, front-left, rear-left, rear-right).
- **Controlling a steering servo** (also over CAN).
- **Managing power sequencing** — low-voltage (LV) and high-voltage (HV) bus bring-up.
- **Computing Ackermann steering geometry** to derive individual wheel speed references from a single velocity and steering angle input.
- **Accepting real-time commands** (velocity and steering angle) from a host PC over UART.

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | STM32F446RE (ARM Cortex-M4, 180 MHz) |
| Board | NUCLEO-F446RE |
| CAN controller | bxCAN (CAN1), 500 kBaud, 87.5% sample point |
| UART debug / command | USART2 & USART3 @ 460800 baud |
| ADC | ADC1, 12-bit — reads two potentiometers (POT1 on PC4 / ADC1_CH14, POT2 on PC5 / ADC1_CH15) |
| Timers | TIM1 (ADC trigger, 250 µs period), TIM3 (PWM for RGB LED), TIM10 (periodic reference send, 1 s period) |
| User I/O | 6 LEDs (LED1–LED6), 4 push-buttons (SW1–SW4) |

### Key GPIO Assignments

| Signal | Port/Pin |
|--------|----------|
| Green (heartbeat) LED | PA5 |
| USR_LED1 | PB2 |
| USR_LED2 | PB1 |
| USR_LED3 | PC2 |
| USR_LED4 | PC1 |
| USR_LED5 (COM error) | PC (configured by HAL) |
| USR_LED6 (fault) | PC (configured by HAL) |
| SW1–SW4 | PB12–PB15 (EXTI, rising+falling) |
| RGB LED green channel | TIM3_CH1 |
| RGB LED red channel | TIM3_CH2 |

---

## System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                  Host PC (optional)                      │
│           UART $V<velocity>\r  /  $S<angle>\r            │
└────────────────────┬─────────────────────────────────────┘
                     │ USART2 @ 460800 baud
┌────────────────────▼──────────────────────────────────────┐
│               STM32F446RE (NUCLEO-F446RE)                 │
│                Central Control Unit (CCU)                 │
│                                                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │  State Machine: START1 → START2 → START3 → DRIVE    │  │
│  └─────────────────────────────────────────────────────┘  │
│  ┌──────────────┐  ┌────────────┐  ┌───────────────────┐  │
│  │ Ackermann    │  │  UART RX   │  │  CAN RX callback  │  │
│  │ geometry     │  │  handler   │  │  (measurements,   │  │
│  │ (per-wheel   │  │  ($V, $S)  │  │   flags, status)  │  │
│  │  velocity)   │  └────────────┘  └───────────────────┘  │
│  └──────────────┘                                         │
└──────────────────────┬────────────────────────────────────┘
                       │ CAN1 @ 500 kBaud
     ┌─────────────────┼────────────────────────┐
     │                 │                        │
┌────▼─────┐    ┌──────▼──────┐          ┌──────▼──────┐
│  Wheel   │    │  Steering   │          │   Power     │
│ Drivers  │    │   Servo     │          │  Controller │
│ ×4       │    │  (class 0D) │          │  (class 0E) │
│(class 0B)│    └─────────────┘          └─────────────┘
└──────────┘
  FR=0x01, FL=0x02, RL=0x03, RR=0x04
```

---

## CAN Bus Protocol

### Bit Timing (500 kBaud)

| Parameter | Value |
|-----------|-------|
| Clock | 45 MHz (APB1) |
| Prescaler | 6 |
| Seg1 | 12 TQ |
| Seg2 | 2 TQ |
| SJW | 1 TQ |
| Sample point | 86.7% |

### Arbitration ID Scheme

CAN IDs are composed from three fields:

```
Bits [10:7]  — class   (device class)
Bits  [6:3]  — device  (instance within class)
Bits  [2:0]  — type    (message type)
```

Helper macro: `CANid(class, device, type)`

#### Device Classes

| Class | Description |
|-------|-------------|
| `0x0B` | Wheel motor driver |
| `0x0D` | Steering servo controller |
| `0x0E` | Power management module |

#### Message Types

| Type | Value | Direction |
|------|-------|-----------|
| COMMAND | `0x00` | CCU → node |
| REFERENCE | `0x02` | CCU → node |
| CONFIG | `0x05` | CCU → node |

#### TX Message IDs (CCU transmits)

| ID | Purpose |
|----|---------|
| `0x011` | POT1 value |
| `0x012` | POT2 value |
| `0x013` | Operation mode |

#### RX Message IDs (CCU receives)

| ID | Source | Content |
|----|--------|---------|
| `0x589 / 0x591 / 0x599 / 0x5A1` | Wheel 1–4 discover response | Discovery ACK → sets `flag.DW1–DW4` |
| `0x689` | Servo discover response | Sets `flag.DSS` |
| `0x70C` | Power controller | LV/HV bus status (bytes 1 & 2) |
| `0x58B / 0x593 / 0x59B / 0x5A3` | Wheel 1–4 measurement | `float[0]` = current (A), `float[1]` = velocity (RPM) |
| `0x68B` | Servo measurement | `int16[0]` = steering angle (raw encoder counts) |
| `0x621 / 0x622` | Remote channel 1/2 | 16-bit value for RGB LED control |
| `0x623` | Remote operation mode | Operation mode byte |

### Key Commands Sent

| Command | Value | Description |
|---------|-------|-------------|
| `CMD_DISCOVER` | `0x90` | Probe a node for presence |
| `CMD_MODE` | `0x10` | Set IDLE (`0x11`) or DRIVE (`0x12`) mode |
| `CMD_DRIVE_STATE` | `0x50` | Set STOPPED (`0x00`) or STARTED (`0x01`) |
| `CMD_VSRV` | `0x20` | Low-voltage servo supply ON/OFF |
| `CMD_HVDC` | `0x30` | High-voltage DC bus ON/OFF |
| `CFG_CONTROL_MODE` | `0x01` | Select TORQUE (`0x01`) or VELOCITY (`0x02`) control |
| Config torque min/max | `0x02` / `0x03` | Current limits: −30 A / +30 A (float, big-endian) |
| Config velocity limit | `0x04` | Velocity limit: 800 RPM (float, big-endian) |

---

## Startup State Machine

The firmware uses a five-state machine (`enum state`) progressing linearly on success or branching to `ERR` on failure:

```
START1 ──(LV & HV OK)──► START2 ──(all nodes found)──► START3 ──(configured)──► DRIVE
  │                          │                              │
  └────────────────(any failure)──────────────────────────► ERR
```

### State Descriptions

| State      | Actions                                                                                                                                              |
|------------|------------------------------------------------------------------------------------------------------------------------------------------------------|
| **START1** | Enables the LV servo supply (`CMD_VSRV ON`), then the HV DC bus (`CMD_HVDC ON`). Waits for power status confirmation via CAN (`flag.LV`, `flag.HV`). |
| **START2** | Broadcasts `CMD_DISCOVER` to all four wheel drivers and the steering servo. Validates discovery flags (`flag.DW1–DW4`, `flag.DSS`).                  |
| **START3** | Configures the steering servo (`SERVO_MODE_START`). Configures all four wheels: velocity control mode, DRIVE mode, STATE_STARTED.                    |
| **DRIVE**  | Starts TIM10 interrupt (1 s period) which periodically executes `Ackermann()` + `SendServoReferenceMsg()` + `UART2_RX()`.                            |
| **ERR**    | Stops all drives (STATE_STOPPED), sets steering servo to idle. Lights USR_LED6 as a fault indicator.                                                 |

State transitions are logged to UART as human-readable messages (e.g. `"State: START2 \n\r"`).

---

## Drive Control — Ackermann Steering

The `Ackermann()` function computes individual wheel speed references from a single velocity reference and the measured steering angle, implementing Ackermann geometry for a four-wheel vehicle.

### Vehicle Parameters

| Parameter                              | Symbol | Value              |
|----------------------------------------|--------|--------------------|
| Wheelbase                              | L      | 0.56 m             |
| Front track offset (front axle to ICR) | b      | 0.33 m             |
| Track width                            | B      | 0.50 m             |
| Wheel radius                           | r      | 0.0385 m (∅ 77 mm) |

### Algorithm

1. The raw encoder angle from the servo is converted to radians:  
   $\gamma = \text{angleSS} \times 0.0219 \times \frac{\pi}{180}$

2. For left or right turns, inner/outer turning radii are computed for all four wheels using the Ackermann constraint plus an empirical cubic correction for the front steering angles.

3. Each wheel's linear velocity is scaled proportionally to its turning radius:  
   $v_i = \frac{r_i}{R} \cdot v_\text{ref}$

4. The velocity in m/s is converted to RPM:  
   $n_i = \frac{v_i}{r_\text{wheel}} \cdot \frac{60}{2\pi}$

5. Reference messages are sent to each wheel driver at 1 ms intervals.

---

## UART Command Interface

The CCU accepts simple ASCII commands on **USART2** (460800 baud, 8N1). Commands are framed with `$` as the start-of-frame character and terminated with `\r` (CR).

| Command                | Format        | Range   | Example     |
|------------------------|---------------|---------|-------------|
| Set velocity reference | `$V<float>\r` | ±10 m/s | `$V1.5\r`   |
| Set steering angle     | `$S<float>\r` | ±50°    | `$S-15.0\r` |

Responses echo the accepted value back (`V=1.500000\r\n`) or reply with `Invalid command\r\n`.

> **Note:** The UART handler runs inside the TIM10 periodic callback, so commands are processed at the reference update rate.

---

## Peripherals & Pin Mapping

### Timers

| Timer | Period | Purpose                                                   |
|-------|--------|-----------------------------------------------------------|
| TIM1  | 250 µs | ADC injection trigger (TRGO)                              |
| TIM3  | —      | PWM output for RGB LED (12-bit, CH1=green, CH2=red)       |
| TIM10 | 1 s    | Periodic control loop: Ackermann + reference TX + UART RX |

### ADC1

| Channel       | Pin | Signal                   |
|---------------|-----|--------------------------|
| Regular CH14  | PC4 | POT1                     |
| Injected CH15 | PC5 | POT2                     |
| Injected CH14 | PC4 | POT1 (injected sequence) |

ADC injected conversions are triggered by TIM1 TRGO.

### UART

| Instance | Baud rate | Role                                        |
|----------|-----------|---------------------------------------------|
| USART2   | 460 800   | Main command port (virtual COM via ST-Link) |
| USART3   | 460 800   | Auxiliary / debug                           |

---

## Project Structure

```
Four_wheel_drive_e-wehicle/
├── README.md
├── configureWheelDrive.txt       # Reference snippet for wheel drive config sequence
├── torque_velocity_limits.txt    # Reference snippet for limit configuration
├── TxData.txt                    # Notes on CAN payload encoding
└── CANtest446/                   # STM32CubeIDE project
    ├── CANtest446.ioc             # STM32CubeMX project file
    ├── STM32F446RETx_FLASH.ld    # Linker script
    ├── startup/
    │   └── startup_stm32f446xx.s # Startup assembly
    ├── Inc/
    │   ├── main.h                # Pin definitions, exported prototypes
    │   ├── adc1_itinj.h          # ADC interrupt / injection module
    │   ├── usart3_itcbuf.h       # USART3 interrupt circular buffer
    │   └── stm32f4xx_it.h        # Interrupt handler declarations
    ├── Src/
    │   ├── main.c                # Application entry point — all vehicle logic
    │   ├── adc1_itinj.c          # ADC interrupt handler and helper functions
    │   ├── usart2_itcbuf.c       # USART2 interrupt-driven circular TX buffer
    │   ├── stm32f4xx_hal_msp.c   # HAL MSP (low-level peripheral init)
    │   ├── stm32f4xx_it.c        # IRQ handlers
    │   └── system_stm32f4xx.c   # System clock configuration table
    ├── Drivers/
    │   ├── CMSIS/                # ARM CMSIS headers
    │   └── STM32F4xx_HAL_Driver/ # ST HAL library (CAN, UART, ADC, TIM, GPIO…)
    └── Doc/
        ├── bittiming.txt         # CAN bit-timing calculation reference
        ├── CANparameters.txt     # Chosen CAN parameters (500 kBaud, 87.5% SP)
        ├── CANdriver.txt         # Migration notes for the new HAL CAN API
        ├── can_loopback.c        # CAN loopback test example
        └── can_network.c/.h      # CAN network test example
```

---

## Building & Flashing

The project is built with **STM32CubeIDE** (Eclipse-based, arm-none-eabi-gcc toolchain).

1. Open STM32CubeIDE and import the project from the `CANtest446/` folder.
2. Select the **Debug** build configuration (or create a Release one).
3. Build: **Project → Build Project** (`Ctrl+B`).
4. Flash: Connect the NUCLEO board via USB, then **Run → Debug** (`F11`) or **Run → Run** (`Ctrl+F11`).

The launch configuration is stored in `CANtest446 Debug.launch`.

> The system clock is configured to **180 MHz** (HSE 8 MHz → PLL × 180 / 4 / 2) with over-drive mode enabled.

---

## Configuration

Key compile-time constants in [CANtest446/Src/main.c](CANtest446/Src/main.c):

| Constant          | Default | Description                                               |
|-------------------|---------|-----------------------------------------------------------|
| `CTRLLOOP_PER`    | 4       | Control loop period multiplier (× TIM1 ticks)             |
| `SWBLOCK_PER`     | 50      | Pushbutton debounce period (× SysTick ms)                 |
| `ERRSIGN_PER`     | 100     | Error LED signal duration (× SysTick ms)                  |
| `INMSG_SIZE`      | 64      | UART receive command buffer size (bytes)                  |
| Velocity limit    | 800 RPM | Sent to wheel drivers at startup via `CFG_VELOCITY_LIMIT` |
| Current limit min | −30 A   | Sent to wheel drivers at startup                          |
| Current limit max | +30 A   | Sent to wheel drivers at startup                          |

## License

Drivers and HAL library: © 2019 STMicroelectronics, BSD 3-Clause License.  
Application code: see source file headers.
