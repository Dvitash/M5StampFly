from ultralytics import YOLO
import cv2
import numpy as np
import requests
import time
import os

# Prefer pulling the stream via the drone proxy so you can stay connected to QU-Device WiFi.
# Set DRONE_HOST (recommended) or DRONE_CAMERA_URL in your environment.
DRONE_HOST = os.getenv("DRONE_HOST", "").strip()
URL = os.getenv("DRONE_CAMERA_URL", "").strip()
if not URL:
    host = DRONE_HOST or "10.0.0.1"
    URL = f"http://{host}/camera/stream"

print("Connecting to:", URL)
r = requests.get(URL, stream=True)
if r.status_code != 200:
    print("❌ HTTP error:", r.status_code)
    raise SystemExit(1)

print("✅ HTTP connected, loading YOLO model...")
model = YOLO("yolov8n.pt")   # small model

print("✅ Model loaded, starting person detection (≈3 FPS)...")
bytes_buf = b""

TARGET_FPS = 3.0
MIN_INTERVAL = 1.0 / TARGET_FPS  # ~0.33s between inferences
last_infer_time = 0.0

for chunk in r.iter_content(chunk_size=2048):
    if not chunk:
        continue

    bytes_buf += chunk

    a = bytes_buf.find(b'\xff\xd8')  # JPEG start
    b = bytes_buf.find(b'\xff\xd9')  # JPEG end

    if a != -1 and b != -1 and b > a:
        jpg = bytes_buf[a:b+2]
        bytes_buf = bytes_buf[b+2:]

        img_array = np.frombuffer(jpg, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        now = time.monotonic()

        # Only run YOLO every MIN_INTERVAL seconds
        if now - last_infer_time >= MIN_INTERVAL:
            last_infer_time = now

            # Optional: resize to reduce load (comment out if you want full res)
            # frame = cv2.resize(frame, (640, 360))

            # YOLOv8 inference, person-only
            results = model(frame, classes=[0], verbose=False)
            annotated = results[0].plot()
        else:
            # Just show the raw frame between inferences
            annotated = frame

        cv2.imshow("YOLOv8 Person Detection (ESP32, ~3 FPS)", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
