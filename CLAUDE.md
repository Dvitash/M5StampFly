# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

M5StampFly is an embedded C/C++ (Arduino) quadcopter flight controller firmware for the M5Stack StampS3 (ESP32-S3). Built with PlatformIO. Originally based on M5Fly-kanazawa/StampFly2024June.

## Build Commands

```bash
pio run                     # Build firmware
pio device monitor          # Serial monitor (115200 baud)
```

CI runs `pio run`, Arduino Lint, and clang-format checks on push/PR.

## Architecture

**Control loop** runs at 400Hz (`loop_400Hz()` in `flight_control.cpp`). Cascade PID structure:

1. **Rate PID** (innermost) — gyro rate control
2. **Angle PID** — orientation from Madgwick AHRS (`sensor.cpp`)
3. **Altitude PID** — Kalman-filtered ToF distance (`alt_kalman.cpp`, `tof.cpp`)
4. **Position PD** — optical flow based X/Y hold

Key source modules in `src/`:
- `flight_control.cpp` — main control loop, motor output, mode state machine
- `rc.cpp` — WiFi control, Supabase polling, embedded web server
- `sensor.cpp` — Madgwick AHRS fusion, Kalman altitude, power monitoring
- `imu.cpp` — BMI270 6-axis IMU via SPI
- `tof.cpp` — VL53LX time-of-flight sensors (bottom altitude + front obstacle)
- `pid.cpp` — PID controller implementations

Custom libraries in `lib/`: `bmi270/`, `MadgwickAHRS/`, `vl53l3c/`.

**Operating modes:** INIT(0), AVERAGE/calibration(1), FLIGHT(2), PARKING(3), LOG(4), AUTO_LANDING(5), FLIP(6).

**Motor pins:** 5, 42, 10, 41 (FL, FR, RL, RR) — 150kHz PWM, 8-bit.

## External Integration

Supabase REST API for drone registry and waypoint polling. Web UI served from embedded `server.html`.

## Code Style

Google-based `.clang-format`. CI enforces formatting.
