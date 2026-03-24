#!/usr/bin/env python3
"""
Raspberry Pi Sender Script (Final – Robust & Automated)
Sends Pixhawk telemetry + camera frames to laptop via UDP
"""

import cv2
import socket
import json
import time
import logging
import argparse
import numpy as np
import os
from pymavlink import mavutil

# ===============================================
# LOGGING
# ===============================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    handlers=[
        logging.FileHandler("/home/pi/drone_sender.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ===============================================
# PIXHAWK TELEMETRY
# ===============================================
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
        try:
            logger.info(f"Connecting to Pixhawk on {self.device} @ {self.baudrate}")
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
            return True
        except Exception as e:
            logger.error(f"Pixhawk connection failed: {e}")
            return False

    def get_telemetry(self):
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

    def close(self):
        if self.master:
            self.master.close()

# ===============================================
# CAMERA
# ===============================================
class CameraCapture:
    def __init__(self, index=0, width=1280, height=720):
        self.index = index
        self.width = width
        self.height = height
        self.cap = None

    def open(self):
        self.cap = cv2.VideoCapture(self.index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            logger.error("Camera failed to open")
            return False

        logger.info("Camera opened successfully")
        return True

    def read(self):
        return self.cap.read()

    def close(self):
        if self.cap:
            self.cap.release()
            self.cap = None
            logger.warning("Camera released")

# ===============================================
# UDP SENDER
# ===============================================
class UDPSender:
    def __init__(self, laptop_ip, telemetry_port, image_port):
        self.addr_telem = (laptop_ip, telemetry_port)
        self.addr_img = (laptop_ip, image_port)

        self.sock_telem = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_img = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_img.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)

    def send_telemetry(self, telemetry):
        try:
            data = json.dumps(telemetry).encode()
            self.sock_telem.sendto(data, self.addr_telem)
            return True
        except Exception as e:
            logger.error(f"Telemetry send error: {e}")
            return False

    def send_image(self, frame, start_quality):
        qualities = [start_quality, 30, 20, 10]

        for q in qualities:
            try:
                ok, enc = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), q]
                )
                if not ok:
                    continue

                data = enc.tobytes()
                if len(data) > 65000:
                    logger.warning(f"JPEG too large @ q={q}")
                    continue

                self.sock_img.sendto(data, self.addr_img)

                if q != start_quality:
                    logger.info(f"Auto-reduced JPEG quality → {q}")

                return True

            except Exception as e:
                logger.error(f"JPEG send failed @ q={q}: {e}")

        return False

    def close(self):
        self.sock_telem.close()
        self.sock_img.close()

# ===============================================
# WEBCAM WAIT
# ===============================================
def wait_for_webcam(device="/dev/video0"):
    logger.info("Waiting for camera Webcam mode...")
    while not os.path.exists(device):
        time.sleep(1)
    logger.info("Webcam detected")

# ===============================================
# MAIN
# ===============================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--laptop-ip", required=True)
    parser.add_argument("--telemetry-port", type=int, default=6000)
    parser.add_argument("--image-port", type=int, default=6001)
    parser.add_argument("--pixhawk-port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--fps", type=float, default=1.0)
    parser.add_argument("--image-quality", type=int, default=40)
    args = parser.parse_args()

    wait_for_webcam()

    pixhawk = PixhawkReader(args.pixhawk_port, args.baudrate)
    camera = CameraCapture(args.camera_index)
    sender = UDPSender(args.laptop_ip, args.telemetry_port, args.image_port)

    if not pixhawk.connect():
        return

    if not camera.open():
        return

    interval = 1.0 / args.fps
    last_send = 0

    try:
        while True:
            if time.time() - last_send < interval:
                time.sleep(0.01)
                continue

            last_send = time.time()

            telemetry = pixhawk.get_telemetry()
            sender.send_telemetry(telemetry)

            ret, frame = camera.read()
            if not ret:
                logger.warning("Frame failed, waiting for webcam again...")
                camera.close()
                wait_for_webcam()
                camera.open()
                continue

            sender.send_image(frame, args.image_quality)

    except KeyboardInterrupt:
        logger.info("Stopping sender")

    finally:
        pixhawk.close()
        camera.close()
        sender.close()

if __name__ == "__main__":
    main()
