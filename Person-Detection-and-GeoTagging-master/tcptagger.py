#!/usr/bin/env python3
import os
import cv2
import math
import json
import yaml
import socket
import struct
import logging
import argparse
import numpy as np
import folium
from ultralytics import YOLO
from dataclasses import dataclass
from datetime import datetime

# =====================================================
# LOGGING
# =====================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(message)s"
)
logger = logging.getLogger(__name__)

# =====================================================
# OUTPUT DIRECTORIES
# =====================================================
BASE = "outputs"
IMAGE_DIR = f"{BASE}/images"
MAP_DIR = f"{BASE}/maps"
WAYPOINT_DIR = f"{BASE}/waypoints"
FINAL_MAP_DIR = f"{BASE}/final_map"

for d in [IMAGE_DIR, MAP_DIR, WAYPOINT_DIR, FINAL_MAP_DIR]:
    os.makedirs(d, exist_ok=True)

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
    frame_id: int
    timestamp: float
    lat: float
    lon: float
    alt_agl: float
    heading_deg: float
    pitch: float
    roll: float

# =====================================================
# CONFIG
# =====================================================
class Config:
    def __init__(self, path):
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)

        self.camera = CameraConfig(
            cfg["camera"]["hfov_deg"],
            cfg["camera"]["vfov_deg"]
        )

        self.model_path = cfg["yolo"]["model_path"]
        self.conf = cfg["yolo"]["confidence_threshold"]
        self.imgsz = cfg["yolo"]["imgsz"]
        self.human_classes = cfg["yolo"]["human_classes"]

        self.dedup_dist = cfg["deduplication"]["min_distance_meters"]
        self.dedup_time = cfg["deduplication"]["time_window_seconds"]

        self.pitch_offset = cfg["gimbal"]["pitch_offset_deg"]

        self.tcp_host = cfg["network"]["tcp_host"]
        self.tcp_port = cfg["network"]["tcp_port"]

        self.max_distance = cfg["validation"]["max_geotag_distance"]

# =====================================================
# GEOMETRY
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

def rotate_ray(ray, heading, pitch, roll, pitch_offset):
    pitch = pitch + pitch_offset
    yaw = math.radians(heading)
    pitch = math.radians(pitch)
    roll = math.radians(roll)

    R = (
        np.array([[math.cos(yaw), -math.sin(yaw), 0],
                  [math.sin(yaw),  math.cos(yaw), 0],
                  [0, 0, 1]])
        @
        np.array([[ math.cos(pitch), 0, math.sin(pitch)],
                  [0, 1, 0],
                  [-math.sin(pitch), 0, math.cos(pitch)]])
        @
        np.array([[1, 0, 0],
                  [0, math.cos(roll), -math.sin(roll)],
                  [0, math.sin(roll),  math.cos(roll)]])
    )
    return R @ ray

def meters_to_latlon(north, east, lat, lon):
    dlat = north / R_EARTH
    dlon = east / (R_EARTH * math.cos(math.radians(lat)))
    return (
        lat + math.degrees(dlat),
        lon + math.degrees(dlon)
    )

def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R_EARTH * 2 * math.asin(math.sqrt(a))

# =====================================================
# WAYPOINT WRITER
# =====================================================
class WaypointWriter:
    def __init__(self, base_lat, base_lon):
        self.file = f"{WAYPOINT_DIR}/survivors_{SESSION_ID}.waypoints"
        self.wps = [
            (0, 1, 0, 16, 0, 0, 0, 0, base_lat, base_lon, RESCUE_ALTITUDE_M, 1)
        ]
        self._save()

    def add(self, lat, lon):
        idx = len(self.wps)
        self.wps.append(
            (idx, 0, 3, 16, 0, 0, 0, 0, lat, lon, RESCUE_ALTITUDE_M, 1)
        )
        self._save()

    def _save(self):
        with open(self.file, "w") as f:
            f.write("QGC WPL 110\n")
            for wp in self.wps:
                f.write("\t".join(map(str, wp)) + "\n")

# =====================================================
# TCP RECEIVER
# =====================================================
class TCPReceiver:
    def __init__(self, host, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((host, port))
        s.listen(1)
        logger.info(f"Waiting TCP {host}:{port}")
        self.conn, addr = s.accept()
        logger.info(f"TCP connected from {addr}")

    def receive(self):
        hdr = self._recv_exact(4)
        if not hdr:
            return None, None

        size = struct.unpack(">I", hdr)[0]
        payload = self._recv_exact(size)

        meta_end = payload.find(b"}") + 1
        meta = json.loads(payload[:meta_end])

        img = cv2.imdecode(
            np.frombuffer(payload[meta_end:], np.uint8),
            cv2.IMREAD_COLOR
        )

        # 🔒 Explicit mapping (ignores jpeg_size safely)
        drone = DroneState(
            frame_id=meta["frame_id"],
            timestamp=meta["timestamp"],
            lat=meta["lat"],
            lon=meta["lon"],
            alt_agl=meta["alt_agl"],
            heading_deg=meta["heading_deg"],
            pitch=meta["pitch"],
            roll=meta["roll"],
        )

        return img, drone

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

# =====================================================
# DEDUP TRACKER
# =====================================================
class DetectionTracker:
    def __init__(self, dist, window):
        self.dist = dist
        self.window = window
        self.items = []

    def is_new(self, lat, lon, ts):
        self.items = [i for i in self.items if ts - i[2] < self.window]
        for la, lo, _ in self.items:
            if haversine(lat, lon, la, lo) < self.dist:
                return False
        self.items.append((lat, lon, ts))
        return True

# =====================================================
# MAIN
# =====================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    cfg = Config(args.config)
    model = YOLO(cfg.model_path)
    rx = TCPReceiver(cfg.tcp_host, cfg.tcp_port)
    tracker = DetectionTracker(cfg.dedup_dist, cfg.dedup_time)

    fmap = None
    wp_writer = None

    while True:
        img, drone = rx.receive()
        if img is None:
            continue

        res = model.predict(
            img,
            conf=cfg.conf,
            imgsz=cfg.imgsz,
            verbose=False
        )[0]

        annotated = img.copy()
        detections = []

        for b in res.boxes:
            cls = res.names[int(b.cls[0])].lower()
            if cls not in cfg.human_classes:
                continue

            x1, y1, x2, y2 = map(int, b.xyxy[0])
            u, v = (x1 + x2) / 2, y2

            ray = pixel_to_ray(
                u, v,
                img.shape[1], img.shape[0],
                cfg.camera.hfov_deg,
                cfg.camera.vfov_deg
            )

            rw = rotate_ray(
                ray,
                drone.heading_deg,
                drone.pitch,
                drone.roll,
                cfg.pitch_offset
            )

            if rw[2] <= 0:
                continue

            t = drone.alt_agl / rw[2]
            north, east = rw[0] * t, rw[1] * t
            dist = math.hypot(north, east)

            if dist > cfg.max_distance:
                continue

            lat, lon = meters_to_latlon(
                north, east,
                drone.lat, drone.lon
            )

            if not tracker.is_new(lat, lon, drone.timestamp):
                continue

            detections.append((lat, lon, dist, (x1, y1, x2, y2)))

        if detections:
            if wp_writer is None:
                wp_writer = WaypointWriter(drone.lat, drone.lon)

            if fmap is None:
                fmap = folium.Map(
                    location=[drone.lat, drone.lon],
                    zoom_start=18
                )

            for lat, lon, dist, (x1, y1, x2, y2) in detections:
                wp_writer.add(lat, lon)

                cv2.rectangle(
                    annotated,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )
                cv2.putText(
                    annotated,
                    f"{dist:.1f} m",
                    (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                folium.CircleMarker(
                    [lat, lon],
                    radius=6,
                    color="red",
                    fill=True
                ).add_to(fmap)

            cv2.imwrite(
                f"{IMAGE_DIR}/{SESSION_ID}_{drone.frame_id}.jpg",
                annotated
            )
            fmap.save(
                f"{MAP_DIR}/survivors_map_{SESSION_ID}.html"
            )

        if args.show:
            cv2.imshow("Frame", annotated)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            elif key == ord("["):
                cfg.conf = max(0.05, cfg.conf - 0.05)
            elif key == ord("]"):
                cfg.conf = min(0.95, cfg.conf + 0.05)
            elif key == ord(","):
                cfg.dedup_dist = max(0.5, cfg.dedup_dist - 0.5)
            elif key == ord("."):
                cfg.dedup_dist += 0.5
            elif key == ord("<"):
                cfg.dedup_time = max(1.0, cfg.dedup_time - 5.0)
            elif key == ord(">"):
                cfg.dedup_time += 5.0
            elif key == ord("-"):
                cfg.pitch_offset -= 1.0
            elif key == ord("="):
                cfg.pitch_offset += 1.0
            elif key == ord("i"):
                logger.info(
                    f"conf={cfg.conf:.2f}, "
                    f"dedup_dist={cfg.dedup_dist:.1f}, "
                    f"dedup_time={cfg.dedup_time:.1f}, "
                    f"pitch_offset={cfg.pitch_offset:.1f}"
                )

    cv2.destroyAllWindows()
    if fmap:
        fmap.save(
            f"{FINAL_MAP_DIR}/final_survivors_map_{SESSION_ID}.html"
        )
    logger.info("Shutdown complete")

if __name__ == "__main__":
    main()
