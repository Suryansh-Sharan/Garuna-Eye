#!/usr/bin/env python3
"""
Raspberry Pi Sender Script
Reads telemetry from Pixhawk via MAVLink and streams images + telemetry to laptop via UDP.
"""

import cv2
import socket
import json
import time
import logging
from pymavlink import mavutil
from datetime import datetime
import numpy as np
import argparse

# ===============================================
# LOGGING SETUP
# ===============================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/home/pi/drone_sender.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ===============================================
# PIXHAWK TELEMETRY READER
# ===============================================
class PixhawkReader:
    """Read telemetry from Pixhawk via MAVLink."""
    
    def __init__(self, connection_string: str = '/dev/ttyAMA0', baudrate: int = 57600):
        """
        Initialize MAVLink connection.
        
        Args:
            connection_string: Serial port or UDP address
                              Examples: '/dev/ttyAMA0', '/dev/ttyUSB0', 'udp:0.0.0.0:14550'
            baudrate: Serial baudrate (typically 57600 or 115200)
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.master = None
        self.last_telemetry = {}
        
    def connect(self):
        """Establish connection to Pixhawk."""
        try:
            logger.info(f"Connecting to Pixhawk: {self.connection_string}")
            self.master = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate
            )
            
            # Wait for heartbeat
            logger.info("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            logger.info(f"Heartbeat received from system {self.master.target_system}, "
                       f"component {self.master.target_component}")
            
            # Request data streams
            self.request_data_streams()
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to Pixhawk: {e}")
            return False
    
    def request_data_streams(self):
        """Request specific MAVLink data streams at desired rates."""
        # Request position, attitude, and GPS data at 10 Hz
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),   # Attitude
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 10),   # VFR_HUD
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 10),
        ]
        
        for stream_id, rate_hz in streams:
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                stream_id,
                rate_hz,
                1  # Start streaming
            )
    
    def get_telemetry(self) -> dict:
        """
        Read current telemetry data.
        
        Returns:
            Dictionary with keys: lat, lon, alt_agl, heading, pitch, roll, timestamp
        """
        # Update telemetry cache with latest messages
        msg = self.master.recv_match(blocking=False)
        
        while msg is not None:
            msg_type = msg.get_type()
            
            # GPS position
            if msg_type == 'GLOBAL_POSITION_INT':
                self.last_telemetry['lat'] = msg.lat / 1e7  # Convert to degrees
                self.last_telemetry['lon'] = msg.lon / 1e7
                self.last_telemetry['alt_agl'] = msg.relative_alt / 1000.0  # mm to meters
                self.last_telemetry['heading'] = msg.hdg / 100.0  # centidegrees to degrees
            
            # Attitude (roll, pitch, yaw)
            elif msg_type == 'ATTITUDE':
                self.last_telemetry['roll'] = np.degrees(msg.roll)
                self.last_telemetry['pitch'] = np.degrees(msg.pitch)
                self.last_telemetry['yaw'] = np.degrees(msg.yaw)
            
            # VFR_HUD for additional data
            elif msg_type == 'VFR_HUD':
                self.last_telemetry['groundspeed'] = msg.groundspeed
                self.last_telemetry['airspeed'] = msg.airspeed
                self.last_telemetry['climb'] = msg.climb
            
            msg = self.master.recv_match(blocking=False)
        
        # Add timestamp
        self.last_telemetry['timestamp'] = time.time()
        
        return self.last_telemetry.copy()
    
    def close(self):
        """Close MAVLink connection."""
        if self.master:
            self.master.close()
            logger.info("Pixhawk connection closed")

# ===============================================
# GIMBAL CONTROL (Optional)
# ===============================================
class GimbalController:
    """Control gimbal pitch via MAVLink."""
    
    def __init__(self, mavlink_connection):
        self.master = mavlink_connection
    
    def set_pitch(self, pitch_deg: float):
        """
        Set gimbal pitch angle.
        
        Args:
            pitch_deg: Desired pitch in degrees (0° = forward, -90° = down)
        """
        # Convert to centidegrees
        pitch_centideg = int(pitch_deg * 100)
        
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
            0,
            pitch_centideg,  # pitch
            0,               # roll
            0,               # yaw
            0, 0, 0,
            mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING
        )
        logger.debug(f"Set gimbal pitch to {pitch_deg}°")

# ===============================================
# CAMERA CAPTURE
# ===============================================
class CameraCapture:
    """Capture images from DJI Action 2 or other camera."""
    
    def __init__(self, camera_index: int = 0, width: int = 1920, height: int = 1080):
        """
        Initialize camera.
        
        Args:
            camera_index: Camera device index (0 for /dev/video0)
            width: Capture width
            height: Capture height
        """
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = None
    
    def open(self):
        """Open camera device."""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            
            # Set FPS (if supported)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            if not self.cap.isOpened():
                raise RuntimeError("Failed to open camera")
            
            logger.info(f"Camera opened: {self.width}x{self.height}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to open camera: {e}")
            return False
    
    def capture_frame(self) -> tuple:
        """
        Capture a single frame.
        
        Returns:
            (success, frame) tuple
        """
        if not self.cap or not self.cap.isOpened():
            return False, None
        
        ret, frame = self.cap.read()
        return ret, frame
    
    def close(self):
        """Release camera."""
        if self.cap:
            self.cap.release()
            logger.info("Camera released")

# ===============================================
# UDP SENDER
# ===============================================
class UDPSender:
    """Send telemetry and images via UDP."""
    
    def __init__(self, laptop_ip: str, telemetry_port: int = 5000, 
                 image_port: int = 5001):
        """
        Initialize UDP sender.
        
        Args:
            laptop_ip: IP address of laptop (Tailscale IP)
            telemetry_port: Port for telemetry data
            image_port: Port for image data
        """
        self.laptop_ip = laptop_ip
        self.telemetry_port = telemetry_port
        self.image_port = image_port
        
        self.telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.image_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        logger.info(f"UDP sender initialized: {laptop_ip}:{telemetry_port}/{image_port}")
    
    def send_telemetry(self, telemetry: dict):
        """Send telemetry as JSON."""
        try:
            # Ensure all required fields are present
            required_fields = ['lat', 'lon', 'alt_agl', 'heading', 'pitch', 'roll']
            if not all(field in telemetry for field in required_fields):
                logger.warning(f"Missing telemetry fields: {set(required_fields) - set(telemetry.keys())}")
                return False
            
            json_data = json.dumps(telemetry).encode('utf-8')
            self.telem_sock.sendto(json_data, (self.laptop_ip, self.telemetry_port))
            logger.debug(f"Sent telemetry: alt={telemetry['alt_agl']:.1f}m, "
                        f"heading={telemetry['heading']:.1f}°")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send telemetry: {e}")
            return False
    
    def send_image(self, frame: np.ndarray, quality: int = 85):
        """
        Send image via UDP.
        
        Args:
            frame: OpenCV image (numpy array)
            quality: JPEG compression quality (0-100)
        """
        try:
            # Encode as JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            result, encoded = cv2.imencode('.jpg', frame, encode_param)
            
            if not result:
                logger.error("Failed to encode image")
                return False
            
            # Send encoded data
            data = encoded.tobytes()
            
            # For large images, might need to split into chunks
            # For simplicity, sending as single packet (works for <64KB)
            if len(data) > 60000:
                logger.warning(f"Image size {len(data)} bytes may exceed UDP limit")
            
            self.image_sock.sendto(data, (self.laptop_ip, self.image_port))
            logger.debug(f"Sent image: {len(data)} bytes")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send image: {e}")
            return False
    
    def close(self):
        """Close UDP sockets."""
        self.telem_sock.close()
        self.image_sock.close()
        logger.info("UDP sockets closed")

# ===============================================
# MAIN LOOP
# ===============================================
def main():
    parser = argparse.ArgumentParser(description="Raspberry Pi Drone Data Sender")
    parser.add_argument('--laptop-ip', type=str, required=True, 
                       help='Laptop IP address (Tailscale IP)')
    parser.add_argument('--telemetry-port', type=int, default=5000,
                       help='UDP port for telemetry')
    parser.add_argument('--image-port', type=int, default=5001,
                       help='UDP port for images')
    parser.add_argument('--pixhawk-port', type=str, default='/dev/ttyAMA0',
                       help='Pixhawk serial port or UDP address')
    parser.add_argument('--baudrate', type=int, default=57600,
                       help='Serial baudrate')
    parser.add_argument('--camera-index', type=int, default=0,
                       help='Camera device index')
    parser.add_argument('--fps', type=float, default=2.0,
                       help='Target frames per second')
    parser.add_argument('--image-quality', type=int, default=85,
                       help='JPEG compression quality (0-100)')
    args = parser.parse_args()
    
    # Initialize components
    pixhawk = PixhawkReader(args.pixhawk_port, args.baudrate)
    camera = CameraCapture(args.camera_index)
    sender = UDPSender(args.laptop_ip, args.telemetry_port, args.image_port)
    
    # Connect to Pixhawk
    if not pixhawk.connect():
        logger.error("Failed to connect to Pixhawk - exiting")
        return
    
    # Open camera
    if not camera.open():
        logger.error("Failed to open camera - exiting")
        pixhawk.close()
        return
    
    logger.info("All systems initialized - starting main loop")
    logger.info(f"Target FPS: {args.fps}")
    
    frame_interval = 1.0 / args.fps
    frame_count = 0
    last_frame_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            # Maintain target FPS
            if current_time - last_frame_time < frame_interval:
                time.sleep(0.01)  # Small sleep to reduce CPU usage
                continue
            
            last_frame_time = current_time
            frame_count += 1
            
            # Get telemetry
            telemetry = pixhawk.get_telemetry()
            
            # Validate telemetry
            if not telemetry.get('lat') or not telemetry.get('lon'):
                logger.warning("Incomplete telemetry - skipping frame")
                continue
            
            # Send telemetry
            sender.send_telemetry(telemetry)
            
            # Capture and send image
            ret, frame = camera.capture_frame()
            if ret:
                sender.send_image(frame, args.image_quality)
                
                if frame_count % 10 == 0:  # Log every 10 frames
                    logger.info(f"Frame #{frame_count} sent - "
                              f"Alt: {telemetry.get('alt_agl', 0):.1f}m, "
                              f"Heading: {telemetry.get('heading', 0):.1f}°, "
                              f"Pitch: {telemetry.get('pitch', 0):.1f}°")
            else:
                logger.warning("Failed to capture frame")
            
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    
    finally:
        # Cleanup
        pixhawk.close()
        camera.close()
        sender.close()
        logger.info("Shutdown complete")

if __name__ == "__main__":
    main()