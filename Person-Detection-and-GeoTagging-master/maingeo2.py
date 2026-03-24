#!/usr/bin/env python3
import os
import cv2
import math
import json
import yaml
import socket
import logging
import argparse
import numpy as np
import folium
from ultralytics import YOLO
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

# =====================================================
# LOGGING
# =====================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# =====================================================
# OUTPUT DIRECTORIES
# =====================================================
BASE_OUTPUT_DIR = "outputs"
IMAGE_DIR = os.path.join(BASE_OUTPUT_DIR, "images")
MAP_DIR = os.path.join(BASE_OUTPUT_DIR, "maps")
WAYPOINT_DIR = os.path.join(BASE_OUTPUT_DIR, "waypoints")
FINAL_MAP_DIR = os.path.join(BASE_OUTPUT_DIR, "final_map")   # >>> ADDITION

os.makedirs(IMAGE_DIR, exist_ok=True)
os.makedirs(MAP_DIR, exist_ok=True)
os.makedirs(WAYPOINT_DIR, exist_ok=True)
os.makedirs(FINAL_MAP_DIR, exist_ok=True)                    # >>> ADDITION

# >>> ADDITION: session id to prevent overwrite between flights
SESSION_ID = datetime.now().strftime("%Y%m%d_%H%M%S")

# =====================================================
# CONSTANTS
# =====================================================
R_EARTH = 6378137.0
RESCUE_ALTITUDE_M = 10.0

# =====================================================
# DATA MODELS
# =====================================================
@dataclass
class CameraConfig:
    hfov_deg: float
    vfov_deg: float

@dataclass
class DroneState:
    lat: float
    lon: float
    alt_agl: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float
    timestamp: float

class Config:
    def __init__(self, path: str):
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)

        self.camera = CameraConfig(
            hfov_deg=cfg["camera"]["hfov_deg"],
            vfov_deg=cfg["camera"]["vfov_deg"]
        )

        self.model_path = cfg["yolo"]["model_path"]
        self.conf = cfg["yolo"]["confidence_threshold"]
        self.imgsz = cfg["yolo"]["imgsz"]
        self.human_classes = cfg["yolo"]["human_classes"]

        self.udp_host = cfg["network"]["udp_host"]
        self.udp_port = cfg["network"]["udp_port"]

        self.max_distance = cfg["validation"]["max_geotag_distance"]

        self.dedup_distance = cfg["deduplication"]["min_distance_meters"]
        self.dedup_time_window = cfg["deduplication"]["time_window_seconds"]

# =====================================================
# GEOMETRY (UNCHANGED)
# =====================================================
def pixel_to_ray(u, v, w, h, hfov, vfov):
    x = (u - w / 2) / (w / 2)
    y = (v - h / 2) / (h / 2)
    ray = np.array([
        math.tan(math.radians(hfov / 2)) * x,
        math.tan(math.radians(vfov / 2)) * y,
        1.0
    ])
    return ray / np.linalg.norm(ray)

def rotate_ray(ray, heading_deg, pitch_deg, roll_deg):
    yaw = math.radians(heading_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)

    R_cam_to_body = np.array([
        [0,  1,  0],
        [1,  0,  0],
        [0,  0,  1],
    ])

    R_roll = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll),  math.cos(roll)]
    ])

    R_pitch = np.array([
        [ math.cos(pitch), 0, math.sin(pitch)],
        [ 0,               1, 0              ],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    R_yaw = np.array([
        [ math.cos(yaw), -math.sin(yaw), 0],
        [ math.sin(yaw),  math.cos(yaw), 0],
        [ 0,              0,             1]
    ])

    return R_yaw @ R_pitch @ R_roll @ (R_cam_to_body @ ray)

def meters_to_latlon(north, east, lat, lon):
    dlat = north / R_EARTH
    dlon = east / (R_EARTH * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)

def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R_EARTH * 2 * math.asin(math.sqrt(a))

# =====================================================
# WAYPOINT WRITER (UNCHANGED)
# =====================================================
class WaypointWriter:
    def __init__(self, base_lat, base_lon):
        self.filename = os.path.join(
            WAYPOINT_DIR,
            f"survivors_{SESSION_ID}.waypoints"   # >>> ADDITION (timestamp only)
        )
        self.waypoints = []
        self.header = "QGC WPL 110\n"

        self.waypoints.append(
            (0, 1, 0, 16, 0, 0, 0, 0, base_lat, base_lon, RESCUE_ALTITUDE_M, 1)
        )
        self._save()

    def add(self, lat, lon):
        idx = len(self.waypoints)
        self.waypoints.append(
            (idx, 0, 3, 16, 0, 0, 0, 0, lat, lon, RESCUE_ALTITUDE_M, 1)
        )
        self._save()

    def _save(self):
        with open(self.filename, "w") as f:
            f.write(self.header)
            for wp in self.waypoints:
                f.write("\t".join(map(str, wp)) + "\n")

# =====================================================
# RECEIVERS (UNCHANGED)
# =====================================================
class TelemetryReceiver:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(1.0)

    def receive(self) -> Optional[DroneState]:
        try:
            data, _ = self.sock.recvfrom(4096)
            t = json.loads(data.decode())
            return DroneState(
                lat=t["lat"],
                lon=t["lon"],
                alt_agl=t["alt_agl"],
                heading_deg=t.get("heading_deg", 0.0),
                pitch_deg=t["pitch"],
                roll_deg=t.get("roll", 0.0),
                timestamp=t.get("timestamp", datetime.now().timestamp())
            )
        except:
            return None

    def close(self):
        self.sock.close()

class ImageReceiver:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(1.0)

    def receive(self):
        try:
            data, _ = self.sock.recvfrom(65536)
            return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        except:
            return None

    def close(self):
        self.sock.close()

# =====================================================
# DEDUPLICATION (UNCHANGED)
# =====================================================
class DetectionTracker:
    def __init__(self, min_dist, window):
        self.min_dist = min_dist
        self.window = window
        self.items = []

    def is_new(self, lat, lon, ts):
        self.items = [d for d in self.items if ts - d[2] < self.window]
        for la, lo, _ in self.items:
            if haversine(lat, lon, la, lo) < self.min_dist:
                return False
        self.items.append((lat, lon, ts))
        return True

# =====================================================
# PEOPLE GEOTAGGER (UNCHANGED)
# =====================================================
class PeopleGeotagger:
    def __init__(self, cfg: Config):
        self.model = YOLO(cfg.model_path)
        self.cfg = cfg
        self.tracker = DetectionTracker(cfg.dedup_distance, cfg.dedup_time_window)

    def detect(self, img):
        res = self.model.predict(img, conf=self.cfg.conf, imgsz=self.cfg.imgsz, verbose=False)[0]
        out = []
        for b in res.boxes:
            cls = res.names[int(b.cls[0])].lower()
            if cls in self.cfg.human_classes:
                out.append(b.xyxy[0].tolist())
        return out

    def geotag(self, boxes, img, drone):
        h, w = img.shape[:2]
        results = []
        for x1, y1, x2, y2 in boxes:
            u, v = (x1 + x2) / 2, y2
            ray = pixel_to_ray(u, v, w, h, self.cfg.camera.hfov_deg, self.cfg.camera.vfov_deg)
            rw = rotate_ray(ray, drone.heading_deg, drone.pitch_deg, drone.roll_deg)

            if rw[2] <= 0:
                continue

            t = drone.alt_agl / rw[2]
            north, east = rw[0]*t, rw[1]*t
            dist = math.hypot(north, east)

            if dist > self.cfg.max_distance:
                continue

            lat, lon = meters_to_latlon(north, east, drone.lat, drone.lon)

            if not self.tracker.is_new(lat, lon, drone.timestamp):
                continue

            results.append((lat, lon, dist, (int(x1), int(y1), int(x2), int(y2))))

        return results

# =====================================================
# MAIN
# =====================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    cfg = Config(args.config)
    geo = PeopleGeotagger(cfg)

    telem = TelemetryReceiver(cfg.udp_host, cfg.udp_port)
    img_rx = ImageReceiver(cfg.udp_host, cfg.udp_port + 1)

    fmap = None
    waypoint_writer = None

    try:
        while True:
            drone = telem.receive()
            img = img_rx.receive()
            if drone is None or img is None:
                continue

            if waypoint_writer is None:
                waypoint_writer = WaypointWriter(drone.lat, drone.lon)

            boxes = geo.detect(img)
            new = geo.geotag(boxes, img, drone)

            if new:
                # >>> ADDITION: save image per detection
                img_name = os.path.join(
                    IMAGE_DIR,
                    f"{SESSION_ID}_{datetime.now().strftime('%H%M%S_%f')}.jpg"
                )
                cv2.imwrite(img_name, img)

                if fmap is None:
                    fmap = folium.Map(location=[drone.lat, drone.lon], zoom_start=18)

                for lat, lon, dist, bbox in new:
                    waypoint_writer.add(lat, lon)
                    folium.CircleMarker([lat, lon], radius=7, color="red", fill=True).add_to(fmap)

                fmap.save(os.path.join(MAP_DIR, f"survivors_map_{SESSION_ID}.html"))

            if args.show:
                cv2.imshow("Live Feed", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    finally:
        telem.close()
        img_rx.close()
        cv2.destroyAllWindows()

        # >>> ADDITION: final aggregated map
        if fmap:
            fmap.save(os.path.join(FINAL_MAP_DIR, f"final_survivors_map_{SESSION_ID}.html"))

        logger.info("Shutdown complete")

if __name__ == "__main__":
    main()
