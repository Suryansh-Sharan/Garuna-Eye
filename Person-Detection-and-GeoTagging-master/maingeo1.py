import math
import cv2
import numpy as np
from ultralytics import YOLO
import folium
import yaml
import logging
import argparse
from dataclasses import dataclass
from typing import List, Optional, Tuple
import socket
import json
from datetime import datetime

# ===============================================
# LOGGING SETUP
# ===============================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('drone_geotag.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# ===============================================
# CONFIGURATION CLASSES
# ===============================================
@dataclass
class CameraConfig:
    hfov_deg: float
    vfov_deg: float
    distortion_coeffs: Optional[np.ndarray] = None
    camera_matrix: Optional[np.ndarray] = None

@dataclass
class DroneState:
    lat: float
    lon: float
    alt_agl: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float = 0.0
    timestamp: float = 0.0

class Config:
    def __init__(self, config_path: str):
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)

        self.camera = CameraConfig(
            hfov_deg=cfg['camera']['hfov_deg'],
            vfov_deg=cfg['camera']['vfov_deg']
        )
        self.model_path = cfg['yolo']['model_path']
        self.confidence_threshold = cfg['yolo']['confidence_threshold']
        self.imgsz = cfg['yolo']['imgsz']
        self.human_classes = cfg['yolo']['human_classes']
        self.udp_host = cfg['network']['udp_host']
        self.udp_port = cfg['network']['udp_port']
        self.buffer_size = cfg['network']['buffer_size']
        self.max_altitude = cfg['validation']['max_altitude']
        self.min_altitude = cfg['validation']['min_altitude']
        self.max_geotag_distance = cfg['validation']['max_geotag_distance']

# ===============================================
# GEO HELPERS
# ===============================================
R_EARTH = 6378137.0

def meters_to_latlon_offset(north_m, east_m, lat0_deg, lon0_deg):
    lat0_rad = math.radians(lat0_deg)
    dlat = north_m / R_EARTH
    dlon = east_m / (R_EARTH * math.cos(lat0_rad))
    return lat0_deg + math.degrees(dlat), lon0_deg + math.degrees(dlon)

def pixel_to_ray(u, v, img_w, img_h, hfov_deg, vfov_deg):
    hfov_rad = math.radians(hfov_deg)
    vfov_rad = math.radians(vfov_deg)
    x_norm = (u - img_w / 2) / (img_w / 2)
    y_norm = (v - img_h / 2) / (img_h / 2)
    x_angle = x_norm * (hfov_rad / 2)
    y_angle = y_norm * (vfov_rad / 2)
    ray = np.array([math.tan(x_angle), math.tan(y_angle), 1.0])
    return ray / np.linalg.norm(ray)

def rotate_camera_to_ned(ray_cam, heading_deg, pitch_deg, roll_deg=0.0):
    yaw = math.radians(heading_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    R_roll = np.array([[math.cos(roll), -math.sin(roll), 0],
                       [math.sin(roll),  math.cos(roll), 0],
                       [0, 0, 1]])
    R_pitch = np.array([[1, 0, 0],
                        [0, math.cos(pitch), -math.sin(pitch)],
                        [0, math.sin(pitch),  math.cos(pitch)]])
    R_yaw = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                      [math.sin(yaw),  math.cos(yaw), 0],
                      [0, 0, 1]])
    R_cam_to_body = np.array([[0, 0, 1],
                              [1, 0, 0],
                              [0, 1, 0]])
    R_total = R_yaw @ R_pitch @ R_roll @ R_cam_to_body
    return R_total @ ray_cam

def intersect_ray_with_ground(ray_ned, altitude):
    if ray_ned[2] <= 0:
        return None
    t = altitude / ray_ned[2]
    return t * ray_ned[0], t * ray_ned[1]

def calculate_geotag_uncertainty(altitude, pitch_deg, pixel_error=5.0):
    pitch_factor = 1.0 / max(abs(math.sin(math.radians(pitch_deg))), 0.1)
    altitude_factor = altitude / 10.0
    return 0.5 + pixel_error * altitude_factor * pitch_factor * 0.1

# ===============================================
# UDP RECEIVERS
# ===============================================
class TelemetryReceiver:
    def __init__(self, host, port, buffer_size=4096):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            logger.info(f"Telemetry receiver listening on {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to create telemetry socket: {e}")
            return False

    def receive_telemetry(self) -> Optional[DroneState]:
        try:
            data, _ = self.sock.recvfrom(self.buffer_size)
            telemetry = json.loads(data.decode('utf-8'))
            return DroneState(
                lat=telemetry['lat'],
                lon=telemetry['lon'],
                alt_agl=telemetry['alt_agl'],
                heading_deg=telemetry['heading'],
                pitch_deg=telemetry['pitch'],
                roll_deg=telemetry.get('roll', 0.0),
                timestamp=telemetry.get('timestamp', datetime.now().timestamp())
            )
        except socket.timeout:
            return None
        except Exception as e:
            logger.error(f"Telemetry receive error: {e}")
            return None

    def close(self):
        if self.sock:
            self.sock.close()

# ===============================================
# IMAGE RECEIVER (FRAMED PACKET VERSION)
# ===============================================
class ImageReceiver:
    def __init__(self, host, port, buffer_size=65536):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None

    def connect(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(2.0)
            logger.info(f"Image receiver listening on {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to create image socket: {e}")
            return False

    def receive_image(self) -> Optional[np.ndarray]:
        """Receive multi-packet image using framed UDP headers."""
        try:
            # Wait for header
            while True:
                header, _ = self.sock.recvfrom(self.buffer_size)
                if not header or header[0:1] != b"H":
                    continue
                num_packets = int(header[1:].decode("utf-8").strip())
                break

            image_data = b""
            packets_received = 0

            while packets_received < num_packets:
                try:
                    packet, _ = self.sock.recvfrom(self.buffer_size)
                    if not packet or packet[0:1] != b"D":
                        continue
                    image_data += packet[1:]
                    packets_received += 1
                except socket.timeout:
                    logger.warning(f"Incomplete image ({packets_received}/{num_packets})")
                    break

            if len(image_data) < 1000:
                logger.warning("Image too small, likely corrupted")
                return None

            nparr = np.frombuffer(image_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if img is None:
                logger.error("Failed to decode image")
                return None

            logger.debug(f"Reassembled image OK: {img.shape}")
            return img

        except socket.timeout:
            logger.warning("Image receive timeout")
            return None
        except Exception as e:
            logger.error(f"Image receive error: {e}")
            return None

    def close(self):
        if self.sock:
            self.sock.close()

# ===============================================
# PEOPLE DETECTION + GEOTAGGING
# ===============================================
class PeopleGeotagger:
    def __init__(self, config: Config):
        self.config = config
        self.model = YOLO(self.config.model_path)
        logger.info(f"YOLO model loaded: {self.config.model_path}")

    def detect_people(self, img: np.ndarray) -> List[dict]:
        results = self.model.predict(
            source=img,
            conf=self.config.confidence_threshold,
            imgsz=self.config.imgsz,
            verbose=False
        )[0]
        detections = []
        for box in results.boxes:
            cls_name = results.names[int(box.cls[0])].lower()
            if cls_name not in self.config.human_classes:
                continue
            x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
            detections.append({
                "class": cls_name,
                "bbox": (x_min, y_min, x_max, y_max),
                "confidence": float(box.conf[0])
            })
        return detections

    def geotag_detections(self, detections, img_shape, drone_state):
        img_h, img_w = img_shape
        geotagged = []
        for det in detections:
            x_min, y_min, x_max, y_max = det['bbox']
            u, v = (x_min + x_max) / 2, y_max
            try:
                ray_cam = pixel_to_ray(u, v, img_w, img_h,
                                       self.config.camera.hfov_deg,
                                       self.config.camera.vfov_deg)
                ray_ned = rotate_camera_to_ned(ray_cam,
                                               drone_state.heading_deg,
                                               drone_state.pitch_deg,
                                               drone_state.roll_deg)
                intersect = intersect_ray_with_ground(ray_ned, drone_state.alt_agl)
                if not intersect:
                    continue
                north_m, east_m = intersect
                lat, lon = meters_to_latlon_offset(north_m, east_m,
                                                   drone_state.lat, drone_state.lon)
                uncertainty = calculate_geotag_uncertainty(drone_state.alt_agl, drone_state.pitch_deg)
                distance = math.sqrt(north_m**2 + east_m**2)
                if distance > self.config.max_geotag_distance:
                    continue
                geotagged.append({
                    **det,
                    "lat": lat,
                    "lon": lon,
                    "distance_m": distance,
                    "uncertainty_m": uncertainty
                })
            except Exception as e:
                logger.error(f"Geotagging failed: {e}")
        return geotagged

# ===============================================
# MAIN PROCESSING
# ===============================================
# def process_frame(img, drone_state, geotagger, config):
#     detections = geotagger.detect_people(img)
#     if not detections:
#         logger.info("No detections")
#         return []
#     geotagged = geotagger.geotag_detections(detections, img.shape[:2], drone_state)
#     logger.info(f"Geotagged {len(geotagged)} people")
#     return geotagged
def process_frame(img, drone_state, geotagger, config):
    detections = geotagger.detect_people(img)
    if not detections:
        logger.info("No detections")
        return []

    geotagged = geotagger.geotag_detections(detections, img.shape[:2], drone_state)
    logger.info(f"Geotagged {len(geotagged)} people")

    # ✅ --- Save annotated image + map ---
    if geotagged:
        annotated = img.copy()
        for det in geotagged:
            x_min, y_min, x_max, y_max = det["bbox"]
            cv2.rectangle(annotated, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 255, 0), 2)
            cv2.putText(annotated,
                        f"{det['class']} {det['confidence']:.2f} ({det['lat']:.6f}, {det['lon']:.6f})",
                        (int(x_min), int(y_min) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        img_path = f"output_frame_{timestamp}.jpg"
        cv2.imwrite(img_path, annotated)
        logger.info(f"✅ Saved annotated image: {img_path}")

        # --- Generate Folium map ---
        m = folium.Map(location=[drone_state.lat, drone_state.lon], zoom_start=18)
        folium.Marker(
            [drone_state.lat, drone_state.lon],
            popup="Drone",
            icon=folium.Icon(color='blue', icon='plane', prefix='fa')
        ).add_to(m)

        for i, det in enumerate(geotagged, start=1):
            folium.CircleMarker(
                location=[det["lat"], det["lon"]],
                radius=6,
                color='red',
                fill=True,
                fill_opacity=0.7,
                popup=f"Person #{i} ({det['lat']:.6f}, {det['lon']:.6f})"
            ).add_to(m)

        map_path = f"geotag_map_{timestamp}.html"
        m.save(map_path)
        logger.info(f"✅ Saved geotag map: {map_path}")

    return geotagged

# ===============================================
# MAIN FUNCTION
# ===============================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, required=True)
    parser.add_argument('--mode', type=str, default='live')
    parser.add_argument('--show', action='store_true', help='Show live frames')
    args = parser.parse_args()

    config = Config(args.config)
    geotagger = PeopleGeotagger(config)

    if args.mode == 'live':
        telem_receiver = TelemetryReceiver(config.udp_host, config.udp_port)
        img_receiver = ImageReceiver(config.udp_host, config.udp_port + 1)
        if not telem_receiver.connect() or not img_receiver.connect():
            logger.error("Receiver setup failed")
            return

        try:
            frame_count = 0
            while True:
                telem = telem_receiver.receive_telemetry()
                if telem is None:
                    continue
                img = img_receiver.receive_image()
                if img is None:
                    continue
                frame_count += 1
                logger.info(f"\n===== Processing frame #{frame_count} =====")
                geotagged = process_frame(img, telem, geotagger, config)

                if args.show:
                    cv2.imshow("Drone Stream", img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        except KeyboardInterrupt:
            logger.info("Shutting down...")
        finally:
            telem_receiver.close()
            img_receiver.close()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
