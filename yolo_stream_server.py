from __future__ import annotations

from dotenv import load_dotenv

load_dotenv()

import os
import time
import uuid
import threading

import cv2
import numpy as np
import requests
import pyttsx3
from flask import Flask, Response, request
from supabase import create_client, Client
from ultralytics import YOLO

# -------------------- CONFIG --------------------
# Prefer pulling the stream via the drone proxy so you can stay connected to QU-Device WiFi.
# Set DRONE_HOST (recommended) or DRONE_CAMERA_URL in your environment.
DRONE_HOST = os.getenv("DRONE_HOST", "").strip()
CAM_URL = os.getenv("DRONE_CAMERA_URL", "").strip()
if not CAM_URL:
    # Fallbacks:
    # - If you set DRONE_HOST to the drone's QU-Device IP (e.g. 10.135.58.60), this will work.
    # - If DRONE_HOST is unset, we default to the old "drone AP" IP.
    host = DRONE_HOST or "10.0.0.1"
    CAM_URL = f"http://192.168.4.1/api/v1/stream"

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

# -------------------- AUDIO --------------------
last_announce_time = 0.0


def speak_person_detected():
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


def handle_person_detection(frame: np.ndarray, num_boxes: int, now: float):
    """
    Handles audio + storage upload + DB alert on detection cooldown.
    Uploads the full frame, not a crop.
    """
    global last_announce_time, last_db_insert_time

    if num_boxes <= 0:
        return

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
        if UPLOAD_MODE:
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

            now = time.monotonic()
            should_run_yolo = (now - last_infer) >= MIN_INTERVAL

            if should_run_yolo:
                last_infer = now
                try:
                    results = model(frame, classes=[0], verbose=False)
                    num_boxes = len(results[0].boxes)

                    # Run alert logic on ORIGINAL frame
                    if num_boxes > 0:
                        handle_person_detection(frame, num_boxes, now)

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

        else:
            try:
                print(f"[YOLO] Connecting to camera: {CAM_URL}")
                r = requests.get(CAM_URL, stream=True, timeout=10)
                if r.status_code != 200:
                    print(f"[YOLO] HTTP error from camera: {r.status_code}")
                    time.sleep(1)
                    continue

                print("[YOLO] Camera HTTP connected, streaming...")
                bytes_buf = b""
                last_frame_time = time.monotonic()

                for chunk in r.iter_content(chunk_size=CHUNK_SIZE):
                    if not chunk:
                        continue

                    # prevent runaway buffer
                    if len(bytes_buf) > MAX_BUFFER_SIZE:
                        jpeg_start = bytes_buf.find(b"\xff\xd8")
                        bytes_buf = bytes_buf[jpeg_start:] if jpeg_start != -1 else b""
                        last_frame_time = time.monotonic()

                    bytes_buf += chunk
                    now_mono = time.monotonic()

                    # frame timeout
                    if now_mono - last_frame_time > FRAME_TIMEOUT:
                        jpeg_start = bytes_buf.find(b"\xff\xd8")
                        bytes_buf = bytes_buf[jpeg_start:] if jpeg_start != -1 else b""
                        last_frame_time = now_mono
                        continue

                    # extract all complete JPGs inside buffer
                    while True:
                        a = bytes_buf.find(b"\xff\xd8")
                        if a == -1:
                            break
                        b = bytes_buf.find(b"\xff\xd9", a + 2)
                        if b == -1:
                            break

                        jpg = bytes_buf[a:b + 2]
                        bytes_buf = bytes_buf[b + 2:]
                        last_frame_time = now_mono

                        img_array = np.frombuffer(jpg, dtype=np.uint8)
                        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                        if frame is None:
                            continue

                        now = time.monotonic()
                        should_run_yolo = (now - last_infer) >= MIN_INTERVAL

                        if should_run_yolo:
                            last_infer = now
                            try:
                                results = model(frame, classes=[0], verbose=False)
                                num_boxes = len(results[0].boxes)

                                # Run alert logic on ORIGINAL frame
                                if num_boxes > 0:
                                    handle_person_detection(frame, num_boxes, now)

                                # draw boxes for stream output
                                frame = results[0].plot()

                            except Exception as e:
                                print("[YOLO] YOLO processing error:", e)

                        ok, jpeg = cv2.imencode(
                            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
                        )
                        if not ok:
                            continue

                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpeg.tobytes()
                            + b"\r\n"
                        )

                print("[YOLO] Camera stream ended, reconnecting...")

            except requests.RequestException as e:
                print(f"[YOLO] Network error, reconnecting in 1s: {e}")
                time.sleep(1)
            except Exception as e:
                print(f"[YOLO] Unexpected error, reconnecting in 1s: {e}")
                time.sleep(1)


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
                print(f"[YOLO] upload FPS: {_upload_fps:.1f}")
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


if __name__ == "__main__":
    bind_host = os.environ.get("YOLO_BIND_HOST", "0.0.0.0")
    bind_port = int(os.environ.get("YOLO_PORT", "5000"))
    print(f"[YOLO] Starting server on {bind_host}:{bind_port}")
    app.run(host=bind_host, port=bind_port, threaded=True, debug=False)