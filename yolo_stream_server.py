from flask import Flask, Response
from ultralytics import YOLO
import cv2
import numpy as np
import requests
import time
import threading
import pyttsx3

CAM_URL = "http://192.168.4.1/api/v1/stream"   # drone → camera stream
TARGET_FPS = 3.0                        # target detections per second
MIN_INTERVAL = 1.0 / TARGET_FPS

app = Flask(__name__)

print("Loading YOLO model...")
model = YOLO("yolov8n.pt")
print("✅ YOLO loaded")

# --- Audio feedback state ---
last_announce_time = 0.0
ANNOUNCE_COOLDOWN = 3.0  # seconds between "person detected" announcements


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


def mjpeg_generator():
    """Yield YOLO-annotated MJPEG frames, auto-reconnecting on errors."""
    global last_announce_time

    MAX_BUFFER_SIZE = 500 * 1024  # 500KB max buffer to prevent memory issues
    CHUNK_SIZE = 8192  # Larger chunk size for better network efficiency
    FRAME_TIMEOUT = 5.0  # Timeout for incomplete frames (seconds)

    while True:
        try:
            print(f"[YOLO] Connecting to camera: {CAM_URL}")
            # Increase timeout for initial connection
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

                # Prevent buffer from growing too large
                if len(bytes_buf) > MAX_BUFFER_SIZE:
                    print("[YOLO] Buffer too large, clearing and seeking new frame...")
                    # Try to find a JPEG start marker and discard everything before it
                    jpeg_start = bytes_buf.find(b"\xff\xd8")
                    if jpeg_start != -1:
                        bytes_buf = bytes_buf[jpeg_start:]
                    else:
                        bytes_buf = b""
                    last_frame_time = time.monotonic()

                bytes_buf += chunk
                current_time = time.monotonic()

                # Timeout check: if we haven't found a complete frame in a while, reset buffer
                if current_time - last_frame_time > FRAME_TIMEOUT:
                    print("[YOLO] Frame timeout, resetting buffer...")
                    jpeg_start = bytes_buf.find(b"\xff\xd8")
                    if jpeg_start != -1:
                        bytes_buf = bytes_buf[jpeg_start:]
                    else:
                        bytes_buf = b""
                    last_frame_time = current_time
                    continue

                # Look for complete JPEG frames
                while True:
                    a = bytes_buf.find(b"\xff\xd8")  # JPEG start
                    if a == -1:
                        break  # No start marker found, wait for more data
                    
                    b = bytes_buf.find(b"\xff\xd9", a + 2)  # JPEG end (start searching after start marker)
                    if b == -1:
                        break  # No end marker found yet, wait for more data

                    # Extract complete JPEG
                    jpg = bytes_buf[a:b+2]
                    bytes_buf = bytes_buf[b+2:]
                    last_frame_time = current_time
                    frame_count += 1

                    try:
                        img_array = np.frombuffer(jpg, dtype=np.uint8)
                        frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                        if frame is None:
                            print(f"[YOLO] Failed to decode frame {frame_count}")
                            continue

                        now = time.monotonic()
                        # Only run YOLO every MIN_INTERVAL seconds, but always yield frames
                        should_run_yolo = (now - last_infer >= MIN_INTERVAL)
                        
                        if should_run_yolo:
                            last_infer = now

                            # Optional: shrink a bit for performance
                            # frame = cv2.resize(frame, (640, 360))

                            try:
                                results = model(frame, classes=[0], verbose=False)
                                frame = results[0].plot()

                                # --- Audio trigger: if any person detected, speak (with cooldown) ---
                                num_boxes = len(results[0].boxes)
                                if num_boxes > 0 and (now - last_announce_time) >= ANNOUNCE_COOLDOWN:
                                    last_announce_time = now
                                    print(f"[YOLO] Person detected ({num_boxes} boxes) → audio")
                                    speak_person_detected()
                            except Exception as yolo_err:
                                print(f"[YOLO] YOLO processing error: {yolo_err}")
                                # Continue with unprocessed frame

                        # Re-encode as JPEG with quality setting for better compression
                        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if not ok:
                            print(f"[YOLO] Failed to encode frame {frame_count}")
                            continue

                        jpg_bytes = jpeg.tobytes()
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" +
                            jpg_bytes +
                            b"\r\n"
                        )
                        
                    except Exception as frame_err:
                        print(f"[YOLO] Frame processing error: {frame_err}")
                        continue

            print("[YOLO] Camera stream ended, reconnecting...")

        except requests.RequestException as e:
            print(f"[YOLO] Network error, reconnecting in 1s: {e}")
            time.sleep(1)
        except Exception as e:
            print(f"[YOLO] Unexpected error, reconnecting in 1s: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(1)
        # while True loop will reconnect


@app.route("/yolo_stream")
def yolo_stream():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)
