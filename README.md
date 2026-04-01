# Fliwatuet — Autonomous Exploration Robot

An Arduino library for building an autonomous, indoor-exploring differential-drive robot on the **Arduino GIGA R1 WiFi**.

Uses a 360° RPLidar A1M8 and a Dynamic Window Approach (DWA) planner with rectangular collision checking to navigate smoothly through unknown environments.

## Hardware

| Component | Detail |
|---|---|
| Board | Arduino GIGA R1 WiFi (STM32H747, Dual-Core M7+M4, Mbed OS) |
| Motor Driver | Arduino Motor Shield Rev3 |
| Motors | 2x Pololu 4751 — 19:1 Metal Gearmotor 12V 37Dx68L, 64 CPR encoder |
| LiDAR | RPLidar A1M8 (360° 2D, standard scan mode) |
| Drive | Differential drive, 70 mm wheels, 210 mm wheel base |

## Wiring

```
Motor Shield Rev3:
  Left  Motor (Ch A) :  DIR=D12   PWM=D3    BRAKE=D9
  Right Motor (Ch B) :  DIR=D13   PWM=D11   BRAKE=D8

Encoders:
  Left  :  A=D4   B=D5
  Right :  A=D6   B=D7

RPLidar A1M8:
  UART      :  Serial1 (D0=RX, D1=TX)
  MOTO_CTRL :  D2

Power:
  12V → Motor Shield VIN (motors)
  USB → GIGA R1 (logic)
  5V  → RPLidar (from GIGA R1 5V pin)
```

## Robot Geometry

All dimensions in mm. Origin = center of drive axle. x = forward, y = left.

```
                 178.38 mm
                   |
        +----------+----------+
        |                     |  113 mm (y+)
        |      (0,0)          |
        |        * axle       |
        |                     | -113 mm (y-)
        +----------+----------+
                   |
                -23.81 mm

  LiDAR center at (80.86, 0) relative to origin
  Outer dimensions: 202.19 x 226 mm
```

## Library Architecture

```
src/
  FliwaConfig.h      — Pin definitions, geometry, kinematic limits, DWA parameters
  FliwaMotor.h/cpp   — Motor Shield driver + PID velocity controller per wheel
  FliwaOdometry.h/cpp— Encoder polling (mbed::Ticker @ 25 kHz) + differential odometry
  FliwaLidar.h/cpp   — RPLidar wrapper, polar→Cartesian transform into robot frame
  FliwaDWA.h/cpp     — Dynamic Window Approach planner with rectangular collision check
  FliwaLogger.h/cpp  — Async WiFi UDP telemetry in a dedicated rtos::Thread
  FliwaRobot.h/cpp   — Top-level state machine (INIT → EXPLORE → RECOVER → ESTOP)
  RPLidarA1Giga.h/cpp— Low-level RPLidar A1 serial protocol driver
```

### Navigation: Dynamic Window Approach

The DWA samples a grid of (v, omega) velocity candidates within the kinematically feasible dynamic window. For each candidate it:

1. Simulates a circular-arc trajectory over a 1.5 s horizon.
2. Transforms every LiDAR scan point into the predicted robot frame.
3. Checks each point against the **inflated rectangular hull** (not a circle) with a 50 mm safety margin.
4. Scores the trajectory via a weighted cost function: obstacle clearance, forward velocity, heading alignment, smoothness.

The planner strongly prefers straight-line driving at maximum speed. It only steers when the forward cone is actually blocked.

### Exploration Strategy

- **Forward clear?** Keep current heading — go straight as fast as possible.
- **Forward blocked?** Find the largest angular gap in the LiDAR scan and steer toward it.
- **Stuck?** If less than 30 mm progress in 4 seconds → enter RECOVER state: stop, do a full 360° scan, rotate toward the largest gap, burst forward, resume EXPLORE.

### Threading Model

| Thread | Rate | Function |
|---|---|---|
| mbed::Ticker ISR | 25 kHz | Encoder quadrature polling |
| Main loop (M7) | 50 Hz PID / 20 Hz nav | Odometry, DWA planning, motor control |
| rtos::Thread | 10 Hz | WiFi UDP telemetry (never blocks nav loop) |

## Quick Start

### 1. Install the library

Symlink this repo into your Arduino libraries folder:

```bash
ln -s /path/to/fliwatuet ~/Documents/Arduino/libraries/fliwatuet
```

### 2. Create your Secrets.h

```bash
cp src/Secrets.h.example examples/FliwatuetExplorer/Secrets.h
```

Edit `examples/FliwatuetExplorer/Secrets.h` and fill in your WiFi credentials:

```cpp
#define WIFI_SSID     "YourNetworkSSID"
#define WIFI_PASSWORD "YourPassword"
#define LOG_TARGET_IP "192.168.1.100"
```

`Secrets.h` is git-ignored and will never be committed.

### 3. Upload

Open `examples/FliwatuetExplorer/FliwatuetExplorer.ino` in the Arduino IDE, select **Arduino GIGA R1 WiFi** as board, and upload.

### 4. Receive telemetry

On your laptop (at the IP you set in `Secrets.h`):

```bash
nc -u -l 9000
```

JSON packets arrive at 10 Hz:

```json
{"t":12345,"x":0.312,"y":-0.041,"th":0.087,"v":0.35,"w":0.02,"tv":5.1,"tw":5.0,"pl":128,"pr":131,"near":423,"st":1,"sc":3.21}
```

### 5. Serial commands

| Key | Action |
|---|---|
| `e` | Emergency Stop |
| `r` | Resume from ESTOP |
| `s` | Print status (pose, velocity, PWM, nearest obstacle) |

## Test Sketches

- **`examples/MotorEncoderTest/`** — Validates motor wiring, encoder counting, and direction conventions. Use this first to verify hardware before running the explorer.
- **`examples/SectorMonitorGiga/`** — Interactive RPLidar sector monitor with configurable angle windows. Use this to verify LiDAR wiring and scan quality.

## Tuning

Key parameters in `src/FliwaConfig.h`:

| Parameter | Default | Effect |
|---|---|---|
| `OP_MAX_V_MS` | 0.40 m/s | Maximum forward speed |
| `DWA_SAFETY_MARGIN_MM` | 50 mm | Clearance added to robot hull |
| `FORWARD_CLEAR_DIST_MM` | 600 mm | Distance threshold for "path is clear" |
| `DWA_WEIGHT_VELOCITY` | 1.8 | Higher = prefers going fast over turning |
| `DWA_WEIGHT_HEADING` | 0.6 | Higher = follows gap heading more aggressively |
| `PID_KP` / `PID_KI` / `PID_KD` | 2.5 / 8.0 / 0.05 | Wheel velocity PID gains |
| `STUCK_TIMEOUT_MS` | 4000 ms | Time without progress before recovery triggers |

## License

MIT
