#!/usr/bin/env python3
"""
Raspberry Pi Sender Script (Corrected)
Sends Pixhawk telemetry + camera frames to laptop via UDP
"""

import cv2
import socket
import json
import time
import logging
import argparse
import numpy as np
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
            logger.info(f"Connecting to Pixhawk on {self.device} @ {self.baudrate} baud")
            self.master = mavutil.mavlink_connection(self.device, baud=self.baudrate)
            self.master.wait_heartbeat(timeout=10)
            logger.info(f"Pixhawk heartbeat received (system {self.master.target_system})")

            # Request data streams
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,  # 10 Hz
                1    # Start
            )
            return True
        except Exception as e:
            logger.error(f"Pixhawk connection failed: {e}")
            return False

    def get_telemetry(self):
        """Read latest telemetry from Pixhawk"""
        msg = self.master.recv_match(blocking=False)
        while msg:
            msg_type = msg.get_type()

            if msg_type == "GLOBAL_POSITION_INT":
                self.last["lat"] = msg.lat / 1e7
                self.last["lon"] = msg.lon / 1e7
                self.last["alt_agl"] = msg.relative_alt / 1000.0
                
                # CRITICAL FIX: heading is in centidegrees
                heading_raw = msg.hdg / 100.0  # Convert to degrees
                
                # Normalize to [0, 360)
                self.last["heading_deg"] = heading_raw % 360.0

            elif msg_type == "ATTITUDE":
                # Convert radians to degrees
                # CRITICAL: Keep pitch sign convention
                # Negative pitch = nose up
                # Positive pitch = nose down
                self.last["roll"] = np.degrees(msg.roll)
                self.last["pitch"] = np.degrees(msg.pitch)

            msg = self.master.recv_match(blocking=False)

        self.last["timestamp"] = time.time()
        return self.last.copy()

    def close(self):
        if self.master:
            self.master.close()
            logger.info("Pixhawk connection closed")

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
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency

        if not self.cap.isOpened():
            logger.error(f"Camera {self.index} failed to open")
            return False

        # Verify resolution
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"Camera opened: {actual_w}x{actual_h}")
        
        return True

    def read(self):
        """Read frame from camera"""
        return self.cap.read()

    def close(self):
        if self.cap:
            self.cap.release()
            logger.info("Camera released")

# ===============================================
# UDP SENDER
# ===============================================
class UDPSender:
    def __init__(self, laptop_ip, telemetry_port, image_port):
        self.addr_telem = (laptop_ip, telemetry_port)
        self.addr_img = (laptop_ip, image_port)
        
        self.sock_telem = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_img = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Increase send buffer
        self.sock_img.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
        
        logger.info(f"UDP sender configured:")
        logger.info(f"  Telemetry -> {laptop_ip}:{telemetry_port}")
        logger.info(f"  Images    -> {laptop_ip}:{image_port}")

    def send_telemetry(self, telemetry):
        """Send telemetry data as JSON"""
        try:
            required = ["lat", "lon", "alt_agl", "heading_deg", "pitch", "roll"]
            if not all(k in telemetry for k in required):
                logger.warning(f"Missing telemetry fields: {[k for k in required if k not in telemetry]}")
                return False
            
            data = json.dumps(telemetry).encode("utf-8")
            self.sock_telem.sendto(data, self.addr_telem)
            return True
            
        except Exception as e:
            logger.error(f"Telemetry send error: {e}")
            return False

    def send_image(self, frame, quality):
        """Encode and send image as JPEG"""
        try:
            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            ok, enc = cv2.imencode(".jpg", frame, encode_param)
            
            if not ok:
                logger.error("JPEG encode failed")
                return False

            jpeg_data = enc.tobytes()
            size = len(jpeg_data)

            # Check size limit
            if size > 65000:
                logger.warning(f"JPEG too large ({size} bytes) - reduce quality or resolution")
                return False

            # Send
            self.sock_img.sendto(jpeg_data, self.addr_img)
            return True

        except Exception as e:
            logger.error(f"Image send error: {e}")
            return False

    def close(self):
        self.sock_telem.close()
        self.sock_img.close()
        logger.info("UDP sockets closed")

# ===============================================
# MAIN
# ===============================================
def main():
    parser = argparse.ArgumentParser(description="Raspberry Pi Drone Data Sender")
    parser.add_argument("--laptop-ip", required=True, help="Laptop Tailscale IP")
    parser.add_argument("--telemetry-port", type=int, default=6000)
    parser.add_argument("--image-port", type=int, default=6001)
    parser.add_argument("--pixhawk-port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=1.0, help="Target FPS")
    parser.add_argument("--image-quality", type=int, default=70, help="JPEG quality (0-100)")
    args = parser.parse_args()

    # Initialize components
    pixhawk = PixhawkReader(args.pixhawk_port, args.baudrate)
    camera = CameraCapture(args.camera_index, args.camera_width, args.camera_height)
    sender = UDPSender(args.laptop_ip, args.telemetry_port, args.image_port)

    # Connect
    if not pixhawk.connect():
        logger.error("Failed to connect to Pixhawk")
        return
    
    if not camera.open():
        logger.error("Failed to open camera")
        return

    # Main loop
    interval = 1.0 / args.fps
    last_send = 0
    frame_id = 0
    
    stats = {
        "frames_sent": 0,
        "frames_failed": 0,
        "telem_sent": 0,
        "telem_failed": 0
    }

    logger.info("=" * 50)
    logger.info("SENDER RUNNING")
    logger.info(f"Target FPS: {args.fps}")
    logger.info(f"JPEG Quality: {args.image_quality}")
    logger.info("Press Ctrl+C to stop")
    logger.info("=" * 50)

    try:
        while True:
            now = time.time()
            
            # Rate limiting
            if now - last_send < interval:
                time.sleep(0.01)
                continue

            last_send = now
            frame_id += 1

            # Get telemetry
            telemetry = pixhawk.get_telemetry()
            
            # Send telemetry
            if sender.send_telemetry(telemetry):
                stats["telem_sent"] += 1
            else:
                stats["telem_failed"] += 1

            # Capture and send image
            ret, frame = camera.read()
            if ret:
                if sender.send_image(frame, args.image_quality):
                    stats["frames_sent"] += 1
                else:
                    stats["frames_failed"] += 1
            else:
                logger.warning("Failed to capture frame")

            # Periodic logging
            if frame_id % 10 == 0:
                logger.info(
                    f"Frame #{frame_id} | "
                    f"GPS: ({telemetry['lat']:.6f}, {telemetry['lon']:.6f}) | "
                    f"Alt: {telemetry['alt_agl']:.1f}m | "
                    f"Hdg: {telemetry['heading_deg']:.0f}° | "
                    f"Pitch: {telemetry['pitch']:.1f}° | "
                    f"Sent: {stats['frames_sent']}/{stats['frames_sent']+stats['frames_failed']}"
                )

    except KeyboardInterrupt:
        logger.info("Stopping sender (Ctrl+C)")

    finally:
        # Cleanup
        pixhawk.close()
        camera.close()
        sender.close()
        
        logger.info("=" * 50)
        logger.info("FINAL STATISTICS")
        logger.info(f"Frames sent: {stats['frames_sent']}")
        logger.info(f"Frames failed: {stats['frames_failed']}")
        logger.info(f"Telemetry sent: {stats['telem_sent']}")
        logger.info(f"Telemetry failed: {stats['telem_failed']}")
        logger.info("=" * 50)

if __name__ == "__main__":
    main()
