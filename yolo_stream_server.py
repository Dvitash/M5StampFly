# yolo_stream_server.py
from flask import Flask, Response, stream_with_context
from ultralytics import YOLO
import cv2
import numpy as np
import requests
import time
import threading
import pyttsx3
import queue
import json

# -------------------- CONFIG --------------------
CAM_URL = "http://192.168.4.1/api/v1/stream"  # drone → camera stream
TARGET_FPS = 3.0
MIN_INTERVAL = 1.0 / TARGET_FPS

ANNOUNCE_COOLDOWN = 3.0   # seconds between voice announcements
EVENT_COOLDOWN = 0.75     # seconds between pushed UI events (prevents spam)

# -------------------- APP --------------------
app = Flask(__name__)

print("Loading YOLO model...")
model = YOLO("yolov8n.pt")
print("✅ YOLO loaded")

# -------------------- AUDIO --------------------
last_announce_time = 0.0


def speak_person_detected():
    """Say 'Person detected' on the computer (in a background thread)."""
    def _worker():
        try:
            engine = pyttsx3.init()
            engine.say("Person detected")
            engine.runAndWait()
        except Exception as e:
            print("[YOLO] TTS error:", e)

    threading.Thread(target=_worker, daemon=True).start()


# -------------------- SSE EVENTS (for React popups) --------------------
events_q: "queue.Queue[dict]" = queue.Queue(maxsize=200)
last_event_push_time = 0.0


def push_event(evt: dict) -> None:
    """Non-blocking enqueue of an event for SSE consumers."""
    try:
        events_q.put_nowait(evt)
    except queue.Full:
        pass


@app.after_request
def add_cors_headers(resp):
    """
    CORS for Vite dev server (http://localhost:8080) calling Flask (http://localhost:5000).
    This is required for EventSource (/events) and is generally fine for dev.
    """
    resp.headers["Access-Control-Allow-Origin"] = "http://localhost:8080"
    resp.headers["Access-Control-Allow-Credentials"] = "true"
    resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
    resp.headers["Access-Control-Allow-Methods"] = "GET,POST,OPTIONS"
    return resp


@app.route("/events")
def events():
    """
    Server-Sent Events endpoint.
    React listens here with EventSource and shows pop-down alerts.
    """
    def gen():
        # initial hello so the client sees a message quickly
        yield f"data: {json.dumps({'type': 'hello', 'ts': time.time()})}\n\n"

        while True:
            try:
                evt = events_q.get(timeout=15)
                yield f"data: {json.dumps(evt)}\n\n"
            except queue.Empty:
                # heartbeat keeps the connection alive
                yield "data: {}\n\n"

    resp = Response(stream_with_context(gen()), mimetype="text/event-stream")
    # Extra SSE-friendly headers (helps with proxies/buffering)
    resp.headers["Cache-Control"] = "no-cache"
    resp.headers["X-Accel-Buffering"] = "no"
    return resp


# -------------------- MJPEG STREAM --------------------
def mjpeg_generator():
    """Yield YOLO-annotated MJPEG frames, auto-reconnecting on errors."""
    global last_announce_time, last_event_push_time

    MAX_BUFFER_SIZE = 500 * 1024
    CHUNK_SIZE = 8192
    FRAME_TIMEOUT = 5.0

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
            frame_count = 0

            for chunk in r.iter_content(chunk_size=CHUNK_SIZE):
                if not chunk:
                    continue

                if len(bytes_buf) > MAX_BUFFER_SIZE:
                    print("[YOLO] Buffer too large, clearing and seeking new frame...")
                    jpeg_start = bytes_buf.find(b"\xff\xd8")
                    bytes_buf = bytes_buf[jpeg_start:] if jpeg_start != -1 else b""
                    last_frame_time = time.monotonic()

                bytes_buf += chunk
                current_time = time.monotonic()

                if current_time - last_frame_time > FRAME_TIMEOUT:
                    print("[YOLO] Frame timeout, resetting buffer...")
                    jpeg_start = bytes_buf.find(b"\xff\xd8")
                    bytes_buf = bytes_buf[jpeg_start:] if jpeg_start != -1 else b""
                    last_frame_time = current_time
                    continue

                while True:
                    a = bytes_buf.find(b"\xff\xd8")
                    if a == -1:
                        break
                    b = bytes_buf.find(b"\xff\xd9", a + 2)
                    if b == -1:
                        break

                    jpg = bytes_buf[a:b + 2]
                    bytes_buf = bytes_buf[b + 2:]
                    last_frame_time = current_time
                    frame_count += 1

                    img_array = np.frombuffer(jpg, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    if frame is None:
                        print(f"[YOLO] Failed to decode frame {frame_count}")
                        continue

                    now = time.monotonic()
                    should_run_yolo = (now - last_infer >= MIN_INTERVAL)

                    if should_run_yolo:
                        last_infer = now
                        try:
                            results = model(frame, classes=[0], verbose=False)
                            frame = results[0].plot()

                            num_boxes = len(results[0].boxes)
                            if num_boxes > 0:
                                # push event to UI (cooldown)
                                if (now - last_event_push_time) >= EVENT_COOLDOWN:
                                    last_event_push_time = now
                                    push_event({
                                        "type": "person_detected",
                                        "boxes": int(num_boxes),
                                        "ts": time.time(),
                                    })

                                # voice (cooldown)
                                if (now - last_announce_time) >= ANNOUNCE_COOLDOWN:
                                    last_announce_time = now
                                    print(f"[YOLO] Person detected ({num_boxes} boxes) → audio")
                                    speak_person_detected()

                        except Exception as yolo_err:
                            print(f"[YOLO] YOLO processing error: {yolo_err}")

                    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if not ok:
                        print(f"[YOLO] Failed to encode frame {frame_count}")
                        continue

                    jpg_bytes = jpeg.tobytes()
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n"
                        + jpg_bytes +
                        b"\r\n"
                    )

            print("[YOLO] Camera stream ended, reconnecting...")

        except requests.RequestException as e:
            print(f"[YOLO] Network error, reconnecting in 1s: {e}")
            time.sleep(1)
        except Exception as e:
            print(f"[YOLO] Unexpected error, reconnecting in 1s: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(1)


@app.route("/yolo_stream")
def yolo_stream():
    resp = Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )
    resp.headers["Cache-Control"] = "no-cache"
    return resp


@app.route("/ping")
def ping():
    return "ok"


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)
