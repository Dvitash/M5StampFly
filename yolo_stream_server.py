from __future__ import annotations

from dotenv import load_dotenv

load_dotenv()

import os
import time
import uuid
import socket
import threading
import concurrent.futures

import cv2
import numpy as np
import requests
try:
    import pyttsx3
    _PYTTSX3_AVAILABLE = True
except Exception:
    _PYTTSX3_AVAILABLE = False
from flask import Flask, Response, request
from supabase import create_client, Client
from ultralytics import YOLO

# -------------------- CONFIG --------------------
# Prefer pulling the stream via the drone proxy so you can stay connected to QU-Device WiFi.
# Set DRONE_HOST (recommended) or DRONE_CAMERA_URL in your environment.
DRONE_HOST = os.getenv("DRONE_HOST", "").strip()
CAM_URL = os.getenv("DRONE_CAMERA_URL", "").strip()
if not CAM_URL:
    # Connect directly to the drone's own AP.
    # If DRONE_HOST is set to a direct drone IP (e.g. 192.168.4.1), use it;
    # otherwise fall back to the standard drone AP address.
    host = DRONE_HOST or "192.168.4.1"
    CAM_URL = f"http://{host}/api/v1/stream"

TARGET_FPS = 3.0
MIN_INTERVAL = 1.0 / TARGET_FPS

ANNOUNCE_COOLDOWN = 3.0       # voice cooldown
DB_ALERT_COOLDOWN = 1.0       # supabase insert cooldown to avoid spam

MAX_BUFFER_SIZE = 500 * 1024
CHUNK_SIZE = 8192
FRAME_TIMEOUT = 5.0

# -------------------- SUPABASE --------------------
SUPABASE_URL = os.getenv("SUPABASE_URL", "").strip()
SUPABASE_SERVICE_ROLE_KEY = os.getenv("SUPABASE_SERVICE_ROLE_KEY", "").strip()
DEFAULT_ORG_ID_STR = os.getenv("DEFAULT_ORG_ID", "").strip()

# Storage settings
DETECTION_IMAGE_BUCKET = "photos"
ACTIVITY_FEED_TABLE = os.getenv("ACTIVITY_FEED_TABLE", "activity_feed").strip()
BACKEND_COMMAND_TABLE = os.getenv("BACKEND_COMMAND_TABLE", "org_drone_priority_state").strip()

# These should match your DB columns if you want the URL/path saved in the row.
ACTIVITY_FEED_IMAGE_URL_COLUMN = os.getenv("ACTIVITY_FEED_IMAGE_URL_COLUMN", "image_url").strip()
ACTIVITY_FEED_IMAGE_PATH_COLUMN = os.getenv("ACTIVITY_FEED_IMAGE_PATH_COLUMN", "image_path").strip()

DEFAULT_ORG_ID: int | None = None
if DEFAULT_ORG_ID_STR.isdigit():
    DEFAULT_ORG_ID = int(DEFAULT_ORG_ID_STR)

supabase: Client | None = None
if SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY:
    supabase = create_client(SUPABASE_URL, SUPABASE_SERVICE_ROLE_KEY)
    print("✅ Supabase client initialized (service role).")
else:
    print("⚠️ Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY. DB inserts and storage uploads disabled.")

if DEFAULT_ORG_ID is None:
    print("⚠️ DEFAULT_ORG_ID not set (or not a number). Inserts will fail if org_id is required.")

# -------------------- APP --------------------
app = Flask(__name__)

# If set, the camera should POST raw JPEG frames to /upload.
# The server will then run YOLO on the latest frame and stream it out.
UPLOAD_MODE = os.getenv("UPLOAD_MODE", "1").strip() != "0"

# Latest camera frame (JPEG bytes) shared across requests
_latest_frame_lock = threading.Lock()
_latest_frame_jpeg: bytes | None = None
_latest_frame_ts = 0.0
_upload_count = 0
_upload_window_start = time.monotonic()
_upload_fps = 0.0

print("Loading YOLO model...")
model = YOLO("yolov8n.pt")
print("✅ YOLO loaded")

# -------------------- PERSON DISTANCE + APPROACH --------------------
# Known person height used for distance estimation (6 feet in meters).
PERSON_HEIGHT_M = 1.8288

# Focal length in pixels for distance estimation.
# For a typical ESP32-CAM at 480p with ~50° vertical FOV:
#   focal_px = (frame_height/2) / tan(VFOV/2) ~= 240 / tan(25°) ~= 515
# Override with CAMERA_FOCAL_LENGTH_PX env var once you have calibrated your camera.
CAMERA_FOCAL_LENGTH_PX = float(os.getenv("CAMERA_FOCAL_LENGTH_PX", "515"))

# ---- Drone IP discovery ----
def _ping_ip(ip: str) -> str | None:
    try:
        r = requests.get(f"http://{ip}/ping", timeout=0.5)
        if r.status_code == 200 and "pong" in r.text.lower():
            return ip
    except Exception:
        pass
    return None

def _find_drone_ip() -> str | None:
    # 1. Try known IPs first (fast, 1.5s timeout each)
    for ip in ([DRONE_HOST] if DRONE_HOST else []) + ["192.168.4.1"]:
        if not ip:
            continue
        try:
            r = requests.get(f"http://{ip}/ping", timeout=1.5)
            if r.status_code == 200 and "pong" in r.text.lower():
                print(f"[drone] Found drone at {ip}")
                return ip
        except Exception:
            pass

    # 2. Scan local subnets in parallel
    try:
        local_ip = socket.gethostbyname(socket.gethostname())
        own_subnet = ".".join(local_ip.split(".")[:3])
    except Exception:
        own_subnet = "10.135.58"

    subnets = list(dict.fromkeys([own_subnet, "10.135.58", "10.135.55", "10.135.57"]))
    all_ips = [f"{s}.{i}" for s in subnets for i in range(1, 255)]
    print(f"[drone] Scanning {len(all_ips)} IPs for drone (this takes ~5s)...")

    with concurrent.futures.ThreadPoolExecutor(max_workers=80) as ex:
        futs = {ex.submit(_ping_ip, ip): ip for ip in all_ips}
        for fut in concurrent.futures.as_completed(futs):
            result = fut.result()
            if result:
                print(f"[drone] Found drone at {result} (scan)")
                for f in futs:
                    f.cancel()
                return result

    print("[drone] WARNING: no drone found — will retry on detection")
    return None

_drone_ip: str | None = _find_drone_ip()
DRONE_API_BASE = f"http://{_drone_ip}" if _drone_ip else ""
print(f"[drone] API base: {DRONE_API_BASE or '(unknown — will scan on detection)'}")

# Approach is triggered only once per server session.
_approach_lock = threading.Lock()
_approach_triggered = False
_consecutive_detection_count = 0
_last_approach_distance_m = 0.0


def estimate_person_distance(bbox_h_px: float, frame_h_px: int) -> float:
    """Estimate distance to a person using the pinhole camera model.

    distance = (known_height_m * focal_length_px) / bbox_height_px

    focal_length_px is scaled to the actual frame height at runtime so the
    estimate stays consistent regardless of capture resolution.
    """
    if bbox_h_px <= 0 or frame_h_px <= 0:
        return -1.0
    # Scale the calibration focal length (for 480 px height) to the actual frame.
    focal_scaled = CAMERA_FOCAL_LENGTH_PX * (frame_h_px / 480.0)
    return (PERSON_HEIGHT_M * focal_scaled) / bbox_h_px


# Retry pacing for the move command.
MOVE_RETRY_INTERVAL_S = 1.0
MOVE_RETRY_MAX_ATTEMPTS = 30  # up to 30 s waiting for flow_valid after airborne

FIXED_APPROACH_DISTANCE_M = float(os.getenv("FIXED_APPROACH_DISTANCE_M", "2.5"))

# How long to wait for the drone to reach FLIGHT_MODE after /hover (s).
TAKEOFF_TIMEOUT_S = float(os.getenv("TAKEOFF_TIMEOUT_S", "20.0"))
# Target hover altitude for takeoff (m).
HOVER_ALTITUDE_M = float(os.getenv("HOVER_ALTITUDE", "0.5"))


def _send_move_once(direction: str, step_m: float) -> tuple[bool, str]:
    """Send one /movement request. Returns (success, reason)."""
    try:
        resp = requests.get(
            f"{DRONE_API_BASE}/movement",
            params={"direction": direction, "step": f"{step_m:.3f}"},
            timeout=5,
        )
    except Exception as e:
        return False, f"request_error:{e}"

    body = resp.text.strip()
    try:
        data = resp.json()
    except Exception:
        # Drone responded with non-JSON (e.g. older firmware returning "OK").
        if resp.status_code == 200 and body.lower() in ("ok", "true"):
            return True, "legacy_ok"
        return False, f"non_json:{resp.status_code}:{body[:80]}"

    if data.get("success") is True:
        return True, "ok"
    return False, str(data.get("reason", f"unknown:{body[:80]}"))


def _telemetry() -> dict | None:
    """Fetch /telemetry from the drone; return parsed dict or None."""
    try:
        r = requests.get(f"{DRONE_API_BASE}/telemetry", timeout=5)
        if r.status_code == 200:
            return r.json()
    except Exception:
        pass
    return None


def request_hover_via_backend() -> bool:
    """Set the backend flag the drone firmware already polls for auto-takeoff."""
    if supabase is None or DEFAULT_ORG_ID is None:
        return False

    payload = {
        "org_id": DEFAULT_ORG_ID,
        "force_scan": True,
        "sweep_intervals": False,
        "disable_temporarily": False,
    }

    try:
        supabase.table(BACKEND_COMMAND_TABLE).upsert(payload).execute()
        print(f"[YOLO] Backend hover request queued: {payload}")
        return True
    except Exception as e:
        print(f"[YOLO] Failed to queue backend hover request: {e}")
        return False


def approach_and_land(distance_m: float):
    """Call /fly directly on the drone — takes off, moves forward, lands."""
    global _approach_triggered, _drone_ip, DRONE_API_BASE

    capped_m = min(distance_m, 4.0)

    # Re-scan if we don't have a working IP yet
    if not _drone_ip:
        print("[YOLO] Drone IP unknown — scanning now...")
        _drone_ip = _find_drone_ip()
        DRONE_API_BASE = f"http://{_drone_ip}" if _drone_ip else ""

    if not _drone_ip:
        print("[YOLO] Still no drone IP — cannot fly. Releasing latch.")
        with _approach_lock:
            _approach_triggered = False
        return

    print(f"[YOLO] Calling http://{_drone_ip}/fly?distance={capped_m:.3f}")

    success = False
    for attempt in range(4):
        try:
            r = requests.get(
                f"http://{_drone_ip}/fly",
                params={"distance": f"{capped_m:.3f}"},
                timeout=3,
            )
            if r.status_code == 200:
                print(f"[YOLO] /fly OK — drone taking off!")
                success = True
                break
            else:
                print(f"[YOLO] /fly attempt {attempt+1}: HTTP {r.status_code} {r.text[:60]}")
        except Exception as e:
            print(f"[YOLO] /fly attempt {attempt+1} failed on {_drone_ip}: {e}")
            # Try re-scanning for a fresh IP
            new_ip = _find_drone_ip()
            if new_ip and new_ip != _drone_ip:
                _drone_ip = new_ip
                DRONE_API_BASE = f"http://{_drone_ip}"
                print(f"[YOLO] Retrying with new IP {_drone_ip}")
        time.sleep(0.5)

    if not success:
        print(f"[YOLO] /fly failed on {_drone_ip} — releasing latch.")
        with _approach_lock:
            _approach_triggered = False
        return

    # Also insert Supabase record as backup / logging
    if supabase and DEFAULT_ORG_ID:
        try:
            occurred_at_iso = time.strftime("%Y-%m-%dT%H:%M:%S.", time.gmtime()) + f"{int(time.time()*1000)%1000:03d}Z"
            supabase.table(ACTIVITY_FEED_TABLE).insert({
                "title": f"CMD_APPROACH_AND_LAND {capped_m:.3f}",
                "occurred_at": occurred_at_iso,
                "org_id": DEFAULT_ORG_ID,
            }).execute()
        except Exception:
            pass

    move_time_s = capped_m / 0.45
    hold_s = 3.0 + 2.0 + move_time_s + 5.0
    print(f"[YOLO] Holding latch {hold_s:.0f}s while drone runs sequence...")
    time.sleep(hold_s)

    with _approach_lock:
        _approach_triggered = False
    print("[YOLO] Latch released — ready for next detection.")


# -------------------- AUDIO --------------------
last_announce_time = 0.0


def speak_person_detected():
    if not _PYTTSX3_AVAILABLE:
        return

    def _worker():
        try:
            engine = pyttsx3.init()
            engine.say("Person detected")
            engine.runAndWait()
        except Exception as e:
            print("[YOLO] TTS error:", e)

    threading.Thread(target=_worker, daemon=True).start()


# -------------------- DB INSERT / STORAGE --------------------
last_db_insert_time = 0.0


def encode_jpeg(frame: np.ndarray, quality: int = 90) -> bytes | None:
    try:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        return buf.tobytes()
    except Exception as e:
        print("[YOLO] JPEG encode error:", e)
        return None


def upload_detection_image(image_bytes: bytes, *, org_id: int) -> tuple[str | None, str | None]:
    """
    Upload full frame JPEG to Supabase Storage.
    Returns (storage_path, public_url).
    public_url is only directly viewable if the bucket is public.
    """
    if supabase is None:
        return None, None

    try:
        now_gmt = time.gmtime()
        date_prefix = time.strftime("%Y/%m/%d", now_gmt)
        filename = f"{int(time.time())}_{uuid.uuid4().hex[:8]}.jpg"
        storage_path = f"org_{org_id}/{date_prefix}/{filename}"

        supabase.storage.from_(DETECTION_IMAGE_BUCKET).upload(
            path=storage_path,
            file=image_bytes,
            file_options={"content-type": "image/jpeg"},
        )

        public_url = None
        try:
            public_url = supabase.storage.from_(DETECTION_IMAGE_BUCKET).get_public_url(storage_path)
        except Exception:
            # Bucket may be private, which is okay.
            public_url = None

        print(f"✅ Uploaded detection frame to storage: {storage_path}")
        return storage_path, public_url

    except Exception as e:
        print("❌ Supabase storage upload failed:", e)
        return None, None


def insert_activity_feed(
    title: str,
    *,
    org_id: int,
    occurred_at_iso: str,
    image_url: str | None = None,
    image_path: str | None = None,
):
    """
    Insert into public.activity_feed:
      - title (varchar)
      - occurred_at (timestamptz)
      - org_id (bigint)
      - optionally image_url / image_path if your schema has them
    """
    if supabase is None:
        return False

    payload = {
        "title": title,
        "occurred_at": occurred_at_iso,
        "org_id": org_id,
    }

    if image_url:
        payload[ACTIVITY_FEED_IMAGE_URL_COLUMN] = image_url
    if image_path:
        payload[ACTIVITY_FEED_IMAGE_PATH_COLUMN] = image_path

    try:
        supabase.table(ACTIVITY_FEED_TABLE).insert(payload).execute()
        print(f"✅ Supabase insert: {payload}")
        return True
    except Exception as e:
        print("❌ Supabase insert failed with image fields:", e)

        # Fallback in case your table does not yet have image columns
        fallback_payload = {
            "title": title,
            "occurred_at": occurred_at_iso,
            "org_id": org_id,
        }
        try:
            supabase.table(ACTIVITY_FEED_TABLE).insert(fallback_payload).execute()
            print(f"✅ Supabase insert succeeded without image fields: {fallback_payload}")
            return True
        except Exception as e2:
            print("❌ Supabase insert fallback failed:", e2)
            return False


def handle_person_detection(frame: np.ndarray, num_boxes: int, now: float, boxes=None):
    """
    Handles distance estimation, audio + storage upload + DB alert on detection cooldown.
    Uploads the full frame, not a crop.
    """
    global last_announce_time, last_db_insert_time, _approach_triggered
    global _consecutive_detection_count, _last_approach_distance_m

    if num_boxes <= 0:
        _consecutive_detection_count = 0
        return

    # ---- Distance estimation & approach ----
    if boxes is not None and len(boxes) > 0:
        frame_h = frame.shape[0]
        max_bbox_h = 0.0
        try:
            xyxy = boxes.xyxy.cpu().numpy()
            for box in xyxy:
                h = float(box[3] - box[1])
                if h > max_bbox_h:
                    max_bbox_h = h
        except Exception as e:
            print(f"[YOLO] Box parsing error: {e}")

        if max_bbox_h > 0:
            dist_m = estimate_person_distance(max_bbox_h, frame_h)
            # only trigger if person is detectable range (>2 m, sensor inaccurate below that)
            if dist_m > 0.3:
                _consecutive_detection_count += 1
                _last_approach_distance_m = dist_m
                print(
                    f"[YOLO] Person detected ({_consecutive_detection_count}/2) — "
                    f"estimated distance: {dist_m:.2f} m "
                    f"(bbox_h={max_bbox_h:.0f} px, frame_h={frame_h} px)"
                )

                # Require 2 consecutive frames before triggering approach
                if _consecutive_detection_count >= 2:
                    with _approach_lock:
                        do_approach = not _approach_triggered
                        if do_approach:
                            _approach_triggered = True

                    if do_approach:
                        _consecutive_detection_count = 0
                        threading.Thread(
                            target=approach_and_land,
                            args=(_last_approach_distance_m,),
                            daemon=True,
                        ).start()
            else:
                _consecutive_detection_count = 0
        else:
            _consecutive_detection_count = 0

    # voice
    if (now - last_announce_time) >= ANNOUNCE_COOLDOWN:
        last_announce_time = now
        print(f"[YOLO] Person detected ({num_boxes}) → audio")
        speak_person_detected()

    # supabase insert (cooldown)
    if (now - last_db_insert_time) >= DB_ALERT_COOLDOWN:
        if DEFAULT_ORG_ID is None:
            print("⚠️ DEFAULT_ORG_ID missing; cannot insert activity_feed row.")
            return

        occurred_at_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        title = f"Person detected ({num_boxes})"

        frame_jpeg = encode_jpeg(frame, quality=90)
        image_path = None
        image_url = None

        if frame_jpeg is not None:
            image_path, image_url = upload_detection_image(
                frame_jpeg,
                org_id=DEFAULT_ORG_ID,
            )

        ok = insert_activity_feed(
            title,
            org_id=DEFAULT_ORG_ID,
            occurred_at_iso=occurred_at_iso,
            image_url=image_url,
            image_path=image_path,
        )

        if ok:
            last_db_insert_time = now


# -------------------- STREAM --------------------
def mjpeg_generator():
    global _latest_frame_jpeg, _latest_frame_ts

    last_infer = 0.0
    last_yolo_frame_ts = 0.0

    while True:
        # Always serve from the shared frame buffer (populated by upload or pull worker)
        with _latest_frame_lock:
            jpeg_data = _latest_frame_jpeg
            frame_ts = _latest_frame_ts

        if jpeg_data is None:
            time.sleep(0.05)
            continue

        # Don't re-process the same frame multiple times
        if frame_ts <= last_yolo_frame_ts:
            time.sleep(0.01)
            continue

        last_yolo_frame_ts = frame_ts

        img_array = np.frombuffer(jpeg_data, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if frame is None:
            continue
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        now = time.monotonic()
        should_run_yolo = (now - last_infer) >= MIN_INTERVAL

        if should_run_yolo:
            last_infer = now
            try:
                results = model(frame, classes=[0], verbose=False)
                num_boxes = len(results[0].boxes)

                # Run alert logic on ORIGINAL frame
                if num_boxes > 0:
                    handle_person_detection(frame, num_boxes, now, boxes=results[0].boxes)

                # draw boxes for stream output
                frame = results[0].plot()

            except Exception as e:
                print("[YOLO] YOLO processing error:", e)

        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            continue

        yield (
            b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpeg.tobytes()
                + b"\r\n"
            )


@app.route("/yolo_stream")
def yolo_stream():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/mjpeg")
def mjpeg_stream_alias():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/upload", methods=["POST"])
def upload_frame():
    """Receive a JPEG frame from the camera and keep it as the latest frame."""
    global _latest_frame_jpeg, _latest_frame_ts
    global _upload_count, _upload_window_start, _upload_fps

    try:
        data = request.get_data()
        if not data:
            return "no data", 400

        # Simple sanity check for JPEG header
        if not data.startswith(b"\xff\xd8"):
            # allow multipart/form-data if camera sends a form field
            # attempt to locate JPEG bytes in the payload
            start = data.find(b"\xff\xd8")
            end = data.rfind(b"\xff\xd9")
            if start == -1 or end == -1 or end <= start:
                print("[YOLO] upload: invalid jpeg payload (len=", len(data), ")")
                return "invalid jpeg", 400
            data = data[start:end + 2]

        with _latest_frame_lock:
            _latest_frame_jpeg = data
            _latest_frame_ts = time.monotonic()
            _upload_count += 1

            now = _latest_frame_ts
            elapsed = now - _upload_window_start
            if elapsed >= 1.0:
                _upload_fps = _upload_count / elapsed
                _upload_count = 0
                _upload_window_start = now

        return "ok", 200
    except Exception as e:
        print("[YOLO] upload error:", e)
        return "error", 500


@app.route("/test_alert")
def test_alert():
    if DEFAULT_ORG_ID is None:
        return "DEFAULT_ORG_ID missing", 400

    occurred_at_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    test_img = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.putText(test_img, "TEST ALERT", (40, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    image_path = None
    image_url = None

    test_jpeg = encode_jpeg(test_img, quality=90)
    if test_jpeg is not None:
        image_path, image_url = upload_detection_image(test_jpeg, org_id=DEFAULT_ORG_ID)

    ok = insert_activity_feed(
        "Person detected (TEST)",
        org_id=DEFAULT_ORG_ID,
        occurred_at_iso=occurred_at_iso,
        image_url=image_url,
        image_path=image_path,
    )
    return ("insert ok" if ok else "insert failed"), (200 if ok else 500)


@app.route("/trigger")
@app.route("/trigger/<float:distance_m>")
def manual_trigger(distance_m: float = 2.5):
    """Manually trigger the approach-and-land sequence.
    Example: http://localhost:5000/trigger        (uses 2.5 m)
             http://localhost:5000/trigger/3.0    (uses 3.0 m)
    """
    global _approach_triggered
    with _approach_lock:
        if _approach_triggered:
            return "approach already in progress — wait for it to finish", 409
        _approach_triggered = True

    threading.Thread(target=approach_and_land, args=(distance_m,), daemon=True).start()
    return f"approach triggered: {distance_m:.2f} m — watch serial monitor", 200


@app.route("/ping")
def ping():
    return "ok"


@app.route("/status")
def status():
    """Return basic info about the last received camera upload."""
    with _latest_frame_lock:
        last_ts = _latest_frame_ts
        upload_fps = _upload_fps
    age = None
    if last_ts > 0:
        age = time.monotonic() - last_ts
    return {
        "last_frame_age_seconds": age,
        "has_frame": last_ts > 0,
        "upload_mode": UPLOAD_MODE,
        "upload_fps": upload_fps,
        "mjpeg_url": "/mjpeg",
        "bucket": DETECTION_IMAGE_BUCKET,
        "activity_feed_table": ACTIVITY_FEED_TABLE,
    }


# -------------------- BACKGROUND DETECTION WORKER --------------------

def _detection_worker():
    """Continuously run YOLO on the latest uploaded frame.

    Runs in a daemon thread started at server launch so detection always
    happens regardless of whether anyone is watching the /yolo_stream.
    """
    last_processed_ts = 0.0
    last_infer = 0.0
    last_frame_warn = 0.0   # rate-limit the "no frames" warning

    while True:
        with _latest_frame_lock:
            jpeg_data = _latest_frame_jpeg
            frame_ts = _latest_frame_ts

        # No frame yet, or same frame as last time
        if jpeg_data is None or frame_ts <= last_processed_ts:
            now = time.monotonic()
            if (now - last_frame_warn) >= 10.0:
                last_frame_warn = now
                print(f"[YOLO] Waiting for camera frames from {CAM_URL} ...")
            time.sleep(0.05)
            continue

        now = time.monotonic()
        if (now - last_infer) < MIN_INTERVAL:
            time.sleep(0.02)
            continue

        last_infer = now
        last_processed_ts = frame_ts

        img_array = np.frombuffer(jpeg_data, dtype=np.uint8)
        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if frame is None:
            continue
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        try:
            results = model(frame, classes=[0], verbose=False)
            num_boxes = len(results[0].boxes)
            if num_boxes > 0:
                handle_person_detection(frame, num_boxes, now, boxes=results[0].boxes)
        except Exception as e:
            print("[YOLO] Detection worker error:", e)


def _pull_camera_worker():
    """Pull MJPEG frames from CAM_URL and store as latest frame for the detection worker."""
    global _latest_frame_jpeg, _latest_frame_ts

    while True:
        try:
            print(f"[CAM] Connecting to {CAM_URL} ...")
            r = requests.get(CAM_URL, stream=True, timeout=10)
            if r.status_code != 200:
                print(f"[CAM] HTTP {r.status_code} from camera, retrying in 3s")
                time.sleep(3)
                continue

            print("[CAM] Connected — pulling frames for detection.")
            buf = b""
            for chunk in r.iter_content(chunk_size=CHUNK_SIZE):
                if not chunk:
                    continue
                if len(buf) > MAX_BUFFER_SIZE:
                    s = buf.find(b"\xff\xd8")
                    buf = buf[s:] if s != -1 else b""
                buf += chunk
                while True:
                    a = buf.find(b"\xff\xd8")
                    if a == -1:
                        break
                    b = buf.find(b"\xff\xd9", a + 2)
                    if b == -1:
                        break
                    jpg = buf[a:b + 2]
                    buf = buf[b + 2:]
                    with _latest_frame_lock:
                        _latest_frame_jpeg = jpg
                        _latest_frame_ts = time.monotonic()
            print("[CAM] Stream ended, reconnecting in 2s...")
            time.sleep(2)

        except Exception as e:
            print(f"[CAM] Error: {e} — retrying in 3s")
            time.sleep(3)


if __name__ == "__main__":
    # Silence Werkzeug's per-request access log (POST /upload spam)
    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    # Detection worker always runs, regardless of UPLOAD_MODE
    t = threading.Thread(target=_detection_worker, daemon=True)
    t.start()
    print("[YOLO] Background detection worker started.")

    # In pull mode, also start a thread that continuously pulls from the camera
    if not UPLOAD_MODE:
        tc = threading.Thread(target=_pull_camera_worker, daemon=True)
        tc.start()
        print(f"[YOLO] Camera pull worker started → {CAM_URL}")
    else:
        print(f"[YOLO] Upload mode ON — waiting for camera to POST frames to /upload")

    bind_host = os.environ.get("YOLO_BIND_HOST", "0.0.0.0")
    bind_port = int(os.environ.get("YOLO_PORT", "5000"))
    print(f"[YOLO] Starting server on {bind_host}:{bind_port}")
    app.run(host=bind_host, port=bind_port, threaded=True, debug=False)