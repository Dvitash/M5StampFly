from ultralytics import YOLO
import cv2
import numpy as np
import requests
import time
import os

# ---------------------------------------------------------------------------
# Config — all values can be overridden with environment variables.
# ---------------------------------------------------------------------------

# Prefer pulling the stream via the drone proxy so you can stay connected to QU-Device WiFi.
# Set DRONE_HOST (recommended) or DRONE_CAMERA_URL in your environment.
DRONE_HOST = os.getenv("DRONE_HOST", "").strip()
URL = os.getenv("DRONE_CAMERA_URL", "").strip()
if not URL:
    host = DRONE_HOST or "10.0.0.1"
    URL = f"http://{host}/camera/stream"

DRONE_BASE_URL = f"http://{DRONE_HOST or '10.0.0.1'}"

# Assumed real-world height of a person (metres) for distance estimation.
PERSON_HEIGHT_M = float(os.getenv("PERSON_HEIGHT_M", "1.7"))

# Camera focal length in pixels.  For a 640-wide frame and ~60° horizontal
# FOV (typical drone cam), this is roughly 533 px.  Tune for your camera.
FOCAL_LENGTH_PX = float(os.getenv("FOCAL_LENGTH_PX", "533"))

# Altitude to hover at after takeoff (metres).
HOVER_ALTITUDE_M = float(os.getenv("HOVER_ALTITUDE", "0.5"))

# Safety cap on how far forward the drone will fly (metres).
MAX_FLY_DISTANCE_M = float(os.getenv("MAX_FLY_DISTANCE_M", "3.0"))

# Minimum YOLO confidence to treat as a valid detection.
MIN_CONF = float(os.getenv("MIN_CONF", "0.50"))

# After triggering an intercept, ignore further detections for this long (s).
TRIGGER_COOLDOWN_S = float(os.getenv("TRIGGER_COOLDOWN_S", "20.0"))

# How long to wait for the drone to reach FLIGHT_MODE after takeoff (s).
TAKEOFF_TIMEOUT_S = float(os.getenv("TAKEOFF_TIMEOUT_S", "15.0"))

# ---------------------------------------------------------------------------
# Drone helpers
# ---------------------------------------------------------------------------

def _drone_get(path, params=None, timeout=5):
    """Send a GET request to the drone web server; return Response or None."""
    try:
        return requests.get(f"{DRONE_BASE_URL}{path}", params=params, timeout=timeout)
    except requests.RequestException as e:
        print(f"  [drone] {path} request error: {e}")
        return None


def _telemetry():
    """Return the /telemetry JSON dict, or None on failure."""
    r = _drone_get("/telemetry")
    if r and r.status_code == 200:
        try:
            return r.json()
        except ValueError:
            pass
    return None


def _estimate_distance(bbox_height_px):
    """Pinhole camera model: distance = (person_height * focal_length) / bbox_height."""
    if bbox_height_px <= 0:
        return None
    return (PERSON_HEIGHT_M * FOCAL_LENGTH_PX) / bbox_height_px


def _trigger_intercept(distance_m):
    """
    Take off, wait for stable flight, then fly forward *distance_m* metres.
    Returns True if the move command was dispatched successfully.
    """
    distance_m = max(0.2, min(distance_m, MAX_FLY_DISTANCE_M))
    print(f"\n[INTERCEPT] Person at ~{distance_m:.1f} m — requesting takeoff + forward move.")

    # 1. Takeoff / hover
    r = _drone_get("/hover", {"altitude": f"{HOVER_ALTITUDE_M:.2f}"})
    if not r or r.status_code != 200:
        print("  [drone] /hover failed — aborting intercept.")
        return False
    print(f"  [drone] Takeoff requested (hover @ {HOVER_ALTITUDE_M:.2f} m).")

    # 2. Poll until the drone is in FLIGHT_MODE (mode==2) with valid optical flow.
    #    FLIGHT_MODE == 2 per the mode state machine in flight_control.cpp.
    deadline = time.monotonic() + TAKEOFF_TIMEOUT_S
    in_flight = False
    while time.monotonic() < deadline:
        tel = _telemetry()
        if tel and tel.get("mode") == 2 and tel.get("flow_valid"):
            in_flight = True
            break
        time.sleep(0.5)

    if not in_flight:
        print("  [drone] Timed out waiting for flight mode / flow — aborting move.")
        return False

    # 3. Issue the forward move.
    print(f"  [drone] In flight — moving forward {distance_m:.2f} m...")
    r = _drone_get("/move", {"direction": "forward", "step": f"{distance_m:.2f}"})
    if r:
        try:
            data = r.json()
            if data.get("success"):
                print(f"  [drone] Move accepted. Target updated.")
            else:
                print(f"  [drone] Move rejected by drone: {data.get('reason')}")
        except ValueError:
            print(f"  [drone] Move response (raw): {r.text}")
    else:
        print("  [drone] /move request failed.")

    return True


# ---------------------------------------------------------------------------
# Main detection loop
# ---------------------------------------------------------------------------

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
last_trigger_time = -TRIGGER_COOLDOWN_S  # allow immediate first trigger
last_results = None  # cache last YOLO output for display between inferences

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

        # Run YOLO at TARGET_FPS; reuse last result between frames.
        if now - last_infer_time >= MIN_INTERVAL:
            last_infer_time = now
            last_results = model(frame, classes=[0], verbose=False)

        if last_results is not None:
            annotated = last_results[0].plot()

            # Check for a triggerable detection (outside cooldown window).
            cooldown_active = (now - last_trigger_time) < TRIGGER_COOLDOWN_S
            best_dist = None
            best_conf = 0.0

            for box in last_results[0].boxes:
                conf = float(box.conf[0])
                if conf < MIN_CONF:
                    continue
                # box.xyxy gives [x1, y1, x2, y2] in pixels
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                bbox_h = y2 - y1
                dist = _estimate_distance(bbox_h)
                if dist is not None and conf > best_conf:
                    best_conf = conf
                    best_dist = dist
                    # Overlay distance on frame
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    cv2.putText(annotated, f"{dist:.1f} m", (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if best_dist is not None and not cooldown_active:
                last_trigger_time = now
                _trigger_intercept(best_dist)
            elif cooldown_active and best_dist is not None:
                remaining = TRIGGER_COOLDOWN_S - (now - last_trigger_time)
                cv2.putText(annotated, f"cooldown {remaining:.0f}s", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            annotated = frame

        cv2.imshow("YOLOv8 Person Detection (ESP32, ~3 FPS)", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

cv2.destroyAllWindows()
