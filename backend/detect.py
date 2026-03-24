import cv2
import socket
import numpy as np
import json
from ultralytics import YOLO
import requests

# ======================================
# CONFIG
# ======================================
VIDEO_PORT = 5001
BACKEND = "http://localhost:8000"
MAX_JPEG_BYTES = 700000

# ======================================
# LOAD YOLO
# ======================================
print("🔵 Loading YOLO model...")
model = YOLO("yolo11s.pt")
print("✅ YOLO ready")

# ======================================
# VIDEO RECEIVER SOCKET
# ======================================
video_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
video_sock.bind(("0.0.0.0", VIDEO_PORT))

buffer = b""

# ======================================
# HTTP HELPERS
# ======================================
def send_frame(endpoint, frame):
    try:
        _, jpeg = cv2.imencode(".jpg", frame)
        requests.post(
            f"{BACKEND}/{endpoint}",
            files={"file": ("frame.jpg", jpeg.tobytes(), "image/jpeg")},
            timeout=0.1
        )
    except:
        pass

def send_detection_flag(person_found: bool):
    """Notify backend if frame has a detection."""
    try:
        requests.post(
            f"{BACKEND}/detect-flag",
            json={"person": person_found},
            timeout=0.1
        )
    except:
        pass

def send_detection_event(lat, lon):
    """Send detection GPS marker to map."""
    try:
        requests.post(
            f"{BACKEND}/detection",
            json={"lat": lat, "lon": lon},
            timeout=0.1
        )
    except:
        pass

# ======================================
# MAIN LOOP
# ======================================
print(f"📥 Listening for video packets on UDP {VIDEO_PORT}")

gps_data = {"lat": None, "lon": None, "alt": None}

while True:
    packet, _ = video_sock.recvfrom(1500)

    if packet != b"FRAME_END":
        buffer += packet
        if len(buffer) > MAX_JPEG_BYTES:
            buffer = b""
        continue

    # Frame complete
    if len(buffer) == 0:
        continue

    frame = cv2.imdecode(np.frombuffer(buffer, np.uint8), cv2.IMREAD_COLOR)
    buffer = b""

    if frame is None:
        continue

    # 1. Send RAW frame to backend
    send_frame("frame", frame)

    # 2. YOLO inference
    results = model(frame)[0]
    person_found = False

    for x1, y1, x2, y2, conf, cls in results.boxes.data:
        if int(cls) == 0:
            person_found = True
            break

    # Notify backend detection state
    send_detection_flag(person_found)

    # Draw only if needed for visual stream
    if person_found:
        for x1, y1, x2, y2, conf, cls in results.boxes.data:
            if int(cls) != 0:
                continue
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 0), 2)

    # 3. Send YOLO frame
    send_frame("detect-frame", frame)
