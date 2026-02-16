from __future__ import annotations
from dotenv import load_dotenv
load_dotenv()


import os
import time
import threading

import cv2
import numpy as np
import requests
import pyttsx3
from flask import Flask, Response
from supabase import create_client, Client
from ultralytics import YOLO

# -------------------- CONFIG --------------------
CAM_URL = "http://192.168.4.1/api/v1/stream"   # drone → camera stream
TARGET_FPS = 3.0
MIN_INTERVAL = 1.0 / TARGET_FPS

ANNOUNCE_COOLDOWN = 3.0     # voice cooldown
DB_ALERT_COOLDOWN = 1.0     # supabase insert cooldown to avoid spam

MAX_BUFFER_SIZE = 500 * 1024
CHUNK_SIZE = 8192
FRAME_TIMEOUT = 5.0

# -------------------- SUPABASE --------------------
SUPABASE_URL = os.getenv("SUPABASE_URL", "").strip()
SUPABASE_SERVICE_ROLE_KEY = os.getenv("SUPABASE_SERVICE_ROLE_KEY", "").strip()
DEFAULT_ORG_ID_STR = os.getenv("DEFAULT_ORG_ID", "").strip()

DEFAULT_ORG_ID: int | None = None
if DEFAULT_ORG_ID_STR.isdigit():
    DEFAULT_ORG_ID = int(DEFAULT_ORG_ID_STR)

supabase: Client | None = None
if SUPABASE_URL and SUPABASE_SERVICE_ROLE_KEY:
    supabase = create_client(SUPABASE_URL, SUPABASE_SERVICE_ROLE_KEY)
    print("✅ Supabase client initialized (service role).")
else:
    print("⚠️ Missing SUPABASE_URL or SUPABASE_SERVICE_ROLE_KEY. DB inserts disabled.")

if DEFAULT_ORG_ID is None:
    print("⚠️ DEFAULT_ORG_ID not set (or not a number). Inserts will fail if org_id is required.")

# -------------------- APP --------------------
app = Flask(__name__)

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


# -------------------- DB INSERT --------------------
last_db_insert_time = 0.0


def insert_activity_feed(title: str, *, org_id: int, occurred_at_iso: str):
    """
    Insert into public.activity_feed:
      - title (varchar)
      - occurred_at (timestamptz)
      - org_id (bigint)
    """
    if supabase is None:
        return False

    payload = {
        "title": title,
        "occurred_at": occurred_at_iso,
        "org_id": org_id,
    }

    try:
        supabase.table("activity_feed").insert(payload).execute()
        print(f"✅ Supabase insert: {payload}")
        return True
    except Exception as e:
        print("❌ Supabase insert failed:", e)
        return False


# -------------------- STREAM --------------------
def mjpeg_generator():
    global last_announce_time, last_db_insert_time

    while True:
        try:
            print(f"[YOLO] Connecting to camera: {CAM_URL}")
            r = requests.get(CAM_URL, stream=True, timeout=10)
            if r.status_code != 200:
                print(f"[YOLO] HTTP error from camera: {r.status_code}")
                time.sleep(1)
                continue

            print("[YOLO] Camera HTTP connected, streaming...")
            bytes_buf = b""
            last_infer = 0.0
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

                    jpg = bytes_buf[a : b + 2]
                    bytes_buf = bytes_buf[b + 2 :]
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

                            # draw boxes
                            frame = results[0].plot()

                            if num_boxes > 0:
                                # voice
                                if (now - last_announce_time) >= ANNOUNCE_COOLDOWN:
                                    last_announce_time = now
                                    print(f"[YOLO] Person detected ({num_boxes}) → audio")
                                    speak_person_detected()

                                # supabase insert (cooldown)
                                if (now - last_db_insert_time) >= DB_ALERT_COOLDOWN:
                                    if DEFAULT_ORG_ID is not None:
                                        title = f"Person detected ({num_boxes})"
                                        occurred_at_iso = time.strftime(
                                            "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
                                        )
                                        ok = insert_activity_feed(
                                            title,
                                            org_id=DEFAULT_ORG_ID,
                                            occurred_at_iso=occurred_at_iso,
                                        )
                                        if ok:
                                            last_db_insert_time = now
                                    else:
                                        print("⚠️ DEFAULT_ORG_ID missing; cannot insert activity_feed row.")

                        except Exception as e:
                            print("[YOLO] YOLO processing error:", e)

                    # encode MJPEG frame
                    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
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


@app.route("/test_alert")
def test_alert():
    if DEFAULT_ORG_ID is None:
        return "DEFAULT_ORG_ID missing", 400

    occurred_at_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    ok = insert_activity_feed(
        "Person detected (TEST)",
        org_id=DEFAULT_ORG_ID,
        occurred_at_iso=occurred_at_iso,
    )
    return ("insert ok" if ok else "insert failed"), (200 if ok else 500)


@app.route("/ping")
def ping():
    return "ok"


if __name__ == "__main__":
    # IMPORTANT: host 0.0.0.0 so other devices can access it via your PC LAN IP
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)
