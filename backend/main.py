import asyncio
import json
import socket
import cv2
import numpy as np
from datetime import datetime
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
import uvicorn
import os

app = FastAPI()

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ======================================
# SAVING CONFIG
# ======================================
SAVE_DIR = "saved_frames"
os.makedirs(SAVE_DIR, exist_ok=True)

SESSION = datetime.now().strftime("%d%b_%I-%M%p")
SESSION_DIR = os.path.join(SAVE_DIR, SESSION)
os.makedirs(SESSION_DIR, exist_ok=True)

print(f"📁 Saving NO-DETECTION frames inside: {SESSION_DIR}")

gps_latest = {"lat": None, "lon": None, "alt": None, "time": None}
last_person_detected = False


def save_no_detection_frame(frame):
    timestamp = datetime.now().strftime("%H%M%S")
    img_name = f"nodetect_{timestamp}.jpg"
    txt_name = f"nodetect_{timestamp}.txt"

    img_path = os.path.join(SESSION_DIR, img_name)
    txt_path = os.path.join(SESSION_DIR, txt_name)

    cv2.imwrite(img_path, frame)

    with open(txt_path, "w") as f:
        f.write(f"Time: {datetime.now().strftime('%H:%M:%S')}\n")
        f.write(f"Image: {img_name}\n\n")
        f.write(f"GPS:\n")
        f.write(f"lat: {gps_latest['lat']}\n")
        f.write(f"lon: {gps_latest['lon']}\n")
        f.write(f"alt: {gps_latest['alt']}\n")

    return img_path


# ======================================
# WEBSOCKET FOR FRONTEND
# ======================================
clients = set()

async def broadcast(msg):
    for ws in list(clients):
        try:
            await ws.send_text(json.dumps(msg))
        except:
            clients.remove(ws)

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    clients.add(ws)
    try:
        while True:
            await ws.receive_text()
    except:
        clients.remove(ws)


# ======================================
# FRAME HOLDERS
# ======================================
latest_frame = None
latest_detect_frame = None


# ======================================
# RAW FRAME RECEIVER
# ======================================
@app.post("/frame")
async def receive_frame(file: UploadFile = File(...)):
    global latest_frame

    data = await file.read()
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    if frame is not None:
        latest_frame = frame

        # Save ONLY if NO person detected in this frame
        if last_person_detected is False:
            save_no_detection_frame(frame)

    return {"status": "ok"}


# ======================================
# DETECTED FRAME RECEIVER
# ======================================
@app.post("/detect-frame")
async def receive_detect_frame(file: UploadFile = File(...)):
    global latest_detect_frame

    data = await file.read()
    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    if frame is not None:
        latest_detect_frame = frame

    return {"status": "ok"}


# ======================================
# PERSON DETECTION FLAG FROM detect.py
# ======================================
@app.post("/detect-flag")
async def detect_flag(payload: dict):
    global last_person_detected

    last_person_detected = payload.get("person", False)
    return {"status": "ok"}


# ======================================
# DETECTION GPS EVENT
# ======================================
@app.post("/detection")
async def post_detection(payload: dict):
    await broadcast({
        "type": "detection",
        "lat": payload["lat"],
        "lon": payload["lon"],
        "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    })
    return {"status": "ok"}


# ======================================
# GPS UDP LISTENER
# ======================================
async def gps_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 6000))
    sock.setblocking(False)

    loop = asyncio.get_event_loop()

    while True:
        try:
            data, _ = await loop.run_in_executor(None, sock.recvfrom, 2048)
            gps = json.loads(data.decode())
            gps_latest.update(gps)

            await broadcast({
                "type": "gps",
                "lat": gps_latest["lat"],
                "lon": gps_latest["lon"],
                "alt": gps_latest["alt"],
                "time": gps_latest["time"],
            })

        except:
            await asyncio.sleep(0.01)


# ======================================
# VIDEO STREAMERS
# ======================================
async def raw_streamer():
    while True:
        if latest_frame is not None:
            ret, jpeg = cv2.imencode(".jpg", latest_frame)
            if ret:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    jpeg.tobytes() + b"\r\n"
                )
        await asyncio.sleep(0.03)

@app.get("/video")
async def video_stream():
    return StreamingResponse(raw_streamer(), media_type="multipart/x-mixed-replace; boundary=frame")


async def detect_streamer():
    while True:
        if latest_detect_frame is not None:
            ret, jpeg = cv2.imencode(".jpg", latest_detect_frame)
            if ret:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    jpeg.tobytes() + b"\r\n"
                )
        await asyncio.sleep(0.03)

@app.get("/detect-stream")
async def detect_stream():
    return StreamingResponse(detect_streamer(), media_type="multipart/x-mixed-replace; boundary=frame")


# ======================================
# STARTUP
# ======================================
@app.on_event("startup")
async def startup():
    asyncio.create_task(gps_listener())
    print("🚀 Backend started with NO-DETECTION saving enabled")

if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
