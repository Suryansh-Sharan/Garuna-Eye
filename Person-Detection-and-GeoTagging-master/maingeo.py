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

class ImageReceiver:
    def __init__(self, host, port, buffer_size=65536):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(2.0)
        logger.info(f"Image receiver listening on {self.host}:{self.port}")
        return True

    def receive_image(self):
        try:
            data, _ = self.sock.recvfrom(self.buffer_size)

            # DEBUG: check size
            if len(data) < 1000:
                logger.warning(f"Received tiny packet: {len(data)} bytes")
                return None

            nparr = np.frombuffer(data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if img is None:
                logger.error("Failed to decode image")
                return None

            return img

        except socket.timeout:
            return None


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
# VISUALIZATION
# ===============================================
def draw_telemetry_overlay(img, drone_state, fps=0):
    """Draw drone telemetry overlay on image."""
    overlay = img.copy()
    h, w = img.shape[:2]
    
    # Semi-transparent background for telemetry
    cv2.rectangle(overlay, (10, 10), (400, 180), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)
    
    # Telemetry text
    font = cv2.FONT_HERSHEY_SIMPLEX
    y_offset = 35
    line_height = 25
    
    telemetry_lines = [
        f"Lat: {drone_state.lat:.6f}",
        f"Lon: {drone_state.lon:.6f}",
        f"Alt: {drone_state.alt_agl:.1f}m",
        f"Heading: {drone_state.heading_deg:.1f}deg",
        f"Pitch: {drone_state.pitch_deg:.1f}deg",
        f"FPS: {fps:.1f}"
    ]
    
    for i, line in enumerate(telemetry_lines):
        cv2.putText(img, line, (20, y_offset + i * line_height),
                   font, 0.6, (0, 255, 0), 2)
    
    return img

def draw_detections(img, geotagged, show_coordinates=True):
    """Draw detection boxes and geotag info on image."""
    for i, det in enumerate(geotagged, start=1):
        x_min, y_min, x_max, y_max = det["bbox"]
        
        # Draw bounding box
        cv2.rectangle(img, (int(x_min), int(y_min)), 
                     (int(x_max), int(y_max)), (0, 255, 0), 2)
        
        # Draw person number
        cv2.circle(img, (int(x_min), int(y_min)), 20, (0, 255, 0), -1)
        cv2.putText(img, str(i), (int(x_min) - 5, int(y_min) + 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # Label with confidence and distance
        label = f"{det['class']} {det['confidence']:.2f}"
        if 'distance_m' in det:
            label += f" | {det['distance_m']:.1f}m"
        
        # Background for text
        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(img, (int(x_min), int(y_min) - text_h - 10),
                     (int(x_min) + text_w + 10, int(y_min)), (0, 255, 0), -1)
        
        cv2.putText(img, label, (int(x_min) + 5, int(y_min) - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Show coordinates if requested
        if show_coordinates and 'lat' in det:
            coord_text = f"({det['lat']:.6f}, {det['lon']:.6f})"
            cv2.putText(img, coord_text, (int(x_min), int(y_max) + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    # Summary at bottom
    if geotagged:
        summary = f"Detected: {len(geotagged)} people"
        cv2.putText(img, summary, (10, img.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    
    return img

# ===============================================
# MAIN PROCESSING
# ===============================================
def process_frame(img, drone_state, geotagger, config, save_map=False):
    """Process frame with detection and geotagging."""
    detections = geotagger.detect_people(img)
    
    if not detections:
        logger.info("No detections")
        return [], img
    
    geotagged = geotagger.geotag_detections(detections, img.shape[:2], drone_state)
    logger.info(f"Geotagged {len(geotagged)} people")
    
    # Create annotated frame
    annotated = draw_detections(img.copy(), geotagged, show_coordinates=True)
    
    # Save map if requested
    if save_map and geotagged:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save annotated image
        img_path = f"output_frame_{timestamp}.jpg"
        cv2.imwrite(img_path, annotated)
        logger.info(f"Saved annotated image: {img_path}")
        
        # Generate Folium map
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
                popup=f"Person #{i}<br>Distance: {det['distance_m']:.1f}m<br>Lat: {det['lat']:.6f}<br>Lon: {det['lon']:.6f}"
            ).add_to(m)
        
        map_path = f"geotag_map_{timestamp}.html"
        m.save(map_path)
        logger.info(f"Saved geotag map: {map_path}")
    
    return geotagged, annotated

# ===============================================
# MAIN FUNCTION
# ===============================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, required=True)
    parser.add_argument('--mode', type=str, default='live')
    parser.add_argument('--show', action='store_true', help='Show live video feed')
    parser.add_argument('--save-interval', type=int, default=10, 
                       help='Save map every N frames (0=never)')
    args = parser.parse_args()

    config = Config(args.config)
    geotagger = PeopleGeotagger(config)

    if args.mode == 'live':
        telem_receiver = TelemetryReceiver(config.udp_host, config.udp_port)
        img_receiver = ImageReceiver(config.udp_host, config.udp_port + 1)
        
        if not telem_receiver.connect() or not img_receiver.connect():
            logger.error("Receiver setup failed")
            return

        logger.info("=" * 60)
        logger.info("LIVE MODE STARTED")
        logger.info("=" * 60)
        if args.show:
            logger.info("Press 'q' to quit")
            logger.info("Press 's' to save current frame + map")
        logger.info("=" * 60)

        try:
            frame_count = 0
            last_time = datetime.now()
            fps = 0
            
            while True:
                # Receive telemetry
                telem = telem_receiver.receive_telemetry()
                if telem is None:
                    continue
                
                # Receive image
                img = img_receiver.receive_image()
                if img is None:
                    continue
                
                frame_count += 1
                
                # Calculate FPS
                current_time = datetime.now()
                time_diff = (current_time - last_time).total_seconds()
                if time_diff > 0:
                    fps = 1.0 / time_diff
                last_time = current_time
                
                logger.info(f"\n{'='*60}")
                logger.info(f"Frame #{frame_count} | FPS: {fps:.1f}")
                
                # Process frame
                save_this_frame = (args.save_interval > 0 and 
                                  frame_count % args.save_interval == 0)
                geotagged, annotated = process_frame(img, telem, geotagger, 
                                                    config, save_map=save_this_frame)
                
                # Show live feed
                if args.show:
                    # Add telemetry overlay
                    display_frame = draw_telemetry_overlay(annotated, telem, fps)
                    
                    # Resize if too large
                    h, w = display_frame.shape[:2]
                    if w > 1280:
                        scale = 1280 / w
                        display_frame = cv2.resize(display_frame, 
                                                  (int(w * scale), int(h * scale)))
                    
                    cv2.imshow("Drone Live Feed - Press 'q' to quit, 's' to save", 
                              display_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        logger.info("Quit requested by user")
                        break
                    elif key == ord('s'):
                        # Manual save
                        logger.info("Manual save requested...")
                        process_frame(img, telem, geotagger, config, save_map=True)
                
        except KeyboardInterrupt:
            logger.info("\nShutting down...")
        # finally:
        #     telem_receiver.close()
        #     img_receiver.close()
        #     if args.show:
        #         display_frame = img.copy()

        #         # Force resize for safety
        #         display_frame = cv2.resize(display_frame, (960, 540))

        #         cv2.imshow("Drone Live Feed", display_frame)

        #         # VERY IMPORTANT on Windows
        #         key = cv2.waitKey(1)
        #         if key == ord('q'):
        #             break
        #         cv2.destroyAllWindows()
        #     logger.info("Shutdown complete")
      
        finally:
            telem_receiver.close()
            img_receiver.close()
            if args.show:
                cv2.destroyAllWindows()
            logger.info("Shutdown complete")


if __name__ == "__main__":
    main()