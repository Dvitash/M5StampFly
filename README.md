# StampFly

## Framework

Platformio

## Base on project

[M5Fly-kanazawa/StampFly2024June (github.com)](https://github.com/M5Fly-kanazawa/StampFly2024June)

## Product introduction

[M5Stampfly](https://docs.m5stack.com/en/app/Stamp%20Fly)

## Third-party libraries

fastled/FastLED

tinyu-zhao/INA3221

mathertel/OneButton @ ^2.5.0

## Camera streaming on QU-Device WiFi (no need to join the drone AP)

The drone firmware runs:

- HTTP API on **port 80** (telemetry/control)
- MJPEG camera proxy on **port 81**

### Browser UI

- Connect your computer to **`QU-Device`** WiFi
- Find the drone's IP on that network (shown on the UI header / your router DHCP list)
- Open `server.html` and set the **IP** field to the drone IP (example `10.135.58.60`)
- Enable **camera stream**

If your camera producer device is not at the default upstream (`192.168.4.1`), set **camera upstream IP** in the UI (or POST to `/camera/upstream`).

### YOLO backend (runs on your PC)

Install deps:

```bash
pip install -r requirements.txt
```

Run:

```bash
set DRONE_HOST=10.135.58.60
python yolo_stream_server.py
```

Then open the YOLO MJPEG endpoint at `http://<your-pc-ip>:5000/yolo_stream`.