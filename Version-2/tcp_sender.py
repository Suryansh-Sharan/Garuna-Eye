#!/usr/bin/env python3
"""
Raspberry Pi TCP Sender 

- Always records flight locally (frames + telemetry)
- Streams to laptop over TCP when available
- Survives network loss, laptop disconnects, no internet
"""

import os
import cv2
import socket
import json
import time
import struct
import logging
import argparse
import numpy as np
from datetime import datetime
from pymavlink import mavutil

# =====================================================
# LOGGING
# =====================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# =====================================================
# PIXHAWK TELEMETRY
# =====================================================
class PixhawkReader:
    def __init__(self, device="/dev/ttyACM0", baudrate=115200):
        self.device = device
        self.baudrate = baudrate
        self.master = None
        self.last = {
            "lat": 0.0,
            "lon": 0.0,
            "alt_agl": 0.0,
            "heading_deg": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
            "timestamp": 0.0
        }

    def connect(self):
        logger.info(f"Connecting Pixhawk on {self.device}")
        self.master = mavutil.mavlink_connection(self.device, baud=self.baudrate)
        self.master.wait_heartbeat(timeout=10)
        logger.info("Pixhawk heartbeat received")

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10,
            1
        )

    def read(self):
        msg = self.master.recv_match(blocking=False)
        while msg:
            t = msg.get_type()

            if t == "GLOBAL_POSITION_INT":
                self.last["lat"] = msg.lat / 1e7
                self.last["lon"] = msg.lon / 1e7
                self.last["alt_agl"] = msg.relative_alt / 1000.0
                self.last["heading_deg"] = (msg.hdg / 100.0) % 360.0

            elif t == "ATTITUDE":
                self.last["roll"] = np.degrees(msg.roll)
                self.last["pitch"] = np.degrees(msg.pitch)

            msg = self.master.recv_match(blocking=False)

        self.last["timestamp"] = time.time()
        return self.last.copy()

# =====================================================
# TCP HELPERS
# =====================================================
def try_connect_tcp(ip, port):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.connect((ip, port))
        s.settimeout(None)
        logger.info("TCP connected to laptop")
        return s
    except Exception:
        return None

# =====================================================
# MAIN
# =====================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--laptop-ip", required=True)
    parser.add_argument("--port", type=int, default=7000)
    parser.add_argument("--fps", type=float, default=1.0)
    parser.add_argument("--quality", type=int, default=70)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--pixhawk-port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()

    # =================================================
    # FLIGHT RECORDING SETUP
    # =================================================
    SESSION_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
    FLIGHT_DIR = f"flights/flight_{SESSION_ID}"
    FRAME_DIR = f"{FLIGHT_DIR}/frames"

    os.makedirs(FRAME_DIR, exist_ok=True)
    log_file = open(f"{FLIGHT_DIR}/telemetry.jsonl", "a")

    logger.info(f"Recording flight to {FLIGHT_DIR}")

    # =================================================
    # CAMERA
    # =================================================
    cam = cv2.VideoCapture(args.camera_index)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cam.isOpened():
        logger.error("Camera failed to open")
        return

    # =================================================
    # PIXHAWK
    # =================================================
    pix = PixhawkReader(args.pixhawk_port, args.baudrate)
    pix.connect()

    # =================================================
    # TCP (OPTIONAL)
    # =================================================
    sock = try_connect_tcp(args.laptop_ip, args.port)
    tcp_connected = sock is not None

    interval = 1.0 / args.fps
    frame_id = 0

    logger.info("Sender running (offline-first mode)")

    try:
        while True:
            start = time.time()

            telem = pix.read()
            if telem["timestamp"] == 0.0:
                time.sleep(0.01)
                continue

            ret, frame = cam.read()
            if not ret:
                time.sleep(0.05)
                continue

            ok, enc = cv2.imencode(
                ".jpg", frame,
                [cv2.IMWRITE_JPEG_QUALITY, args.quality]
            )
            if not ok:
                continue

            jpeg = enc.tobytes()

            # -----------------------------------------
            # SAVE LOCALLY (ALWAYS)
            # -----------------------------------------
            frame_name = f"frame_{frame_id:06d}.jpg"
            with open(f"{FRAME_DIR}/{frame_name}", "wb") as f:
                f.write(jpeg)

            log_entry = {
                "frame_id": frame_id,
                "timestamp": telem["timestamp"],
                "image": f"frames/{frame_name}",
                "lat": telem["lat"],
                "lon": telem["lon"],
                "alt_agl": telem["alt_agl"],
                "heading_deg": telem["heading_deg"],
                "pitch": telem["pitch"],
                "roll": telem["roll"]
            }

            log_file.write(json.dumps(log_entry) + "\n")
            log_file.flush()  # CRASH SAFE

            # -----------------------------------------
            # SEND OVER TCP (BEST EFFORT)
            # -----------------------------------------
            if tcp_connected:
                meta = log_entry.copy()
                meta["jpeg_size"] = len(jpeg)
                payload = json.dumps(meta).encode("utf-8") + jpeg
                try:
                    sock.sendall(struct.pack(">I", len(payload)) + payload)
                except Exception:
                    logger.warning("TCP lost — switching to offline mode")
                    sock.close()
                    sock = None
                    tcp_connected = False

            # -----------------------------------------
            # RECONNECT ATTEMPT (EVERY 30 FRAMES)
            # -----------------------------------------
            if not tcp_connected and frame_id % 30 == 0:
                sock = try_connect_tcp(args.laptop_ip, args.port)
                tcp_connected = sock is not None

            logger.info(
                f"Frame {frame_id} | "
                f"GPS=({telem['lat']:.6f},{telem['lon']:.6f}) "
                f"Alt={telem['alt_agl']:.1f}m | "
                f"TCP={'ON' if tcp_connected else 'OFF'}"
            )

            frame_id += 1

            sleep = interval - (time.time() - start)
            if sleep > 0:
                time.sleep(sleep)

    except KeyboardInterrupt:
        logger.info("Sender stopped")

    finally:
        cam.release()
        if sock:
            sock.close()
        log_file.close()
        logger.info("Shutdown complete")

# =====================================================
if __name__ == "__main__":
    main()

