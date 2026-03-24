#!/usr/bin/env python3
"""
Offline Flight Processor

Replays a recorded flight (frames + telemetry)
and generates:
- survivor images
- maps
- waypoints
- final combined map
"""

import os
import cv2
import math
import json
import yaml
import logging
import argparse
import numpy as np
import folium
from dataclasses import dataclass
from datetime import datetime
from ultralytics import YOLO

# =====================================================
# LOGGING
# =====================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(message)s"
)
logger = logging.getLogger(__name__)

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
    pitch += pitch_offset
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
    return lat + math.degrees(dlat), lon + math.degrees(dlon)

def haversine(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R_EARTH * 2 * math.asin(math.sqrt(a))

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

class CentroidClusterer:
    def __init__(self, radius_m):
        self.radius = radius_m
        self.clusters = []  # [lat, lon, count]

    def add(self, lat, lon):
        """
        Returns:
        (centroid_lat, centroid_lon, is_new_cluster)
        """
        for c in self.clusters:
            d = haversine(lat, lon, c[0], c[1])
            if d < self.radius:
                # running centroid update
                c[2] += 1
                c[0] = c[0] + (lat - c[0]) / c[2]
                c[1] = c[1] + (lon - c[1]) / c[2]
                return c[0], c[1], False

        # new cluster
        self.clusters.append([lat, lon, 1])
        return lat, lon, True


# =====================================================
# MAIN
# =====================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--flight-dir", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", default="offline_outputs")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    img_dir = f"{args.out}/images"
    map_dir = f"{args.out}/maps"
    wp_dir = f"{args.out}/waypoints"
    final_dir = f"{args.out}/final_map"

    for d in [img_dir, map_dir, wp_dir, final_dir]:
        os.makedirs(d, exist_ok=True)

    cfg = Config(args.config)
    model = YOLO(cfg.model_path)
    tracker = DetectionTracker(cfg.dedup_dist, cfg.dedup_time)
    clusterer = CentroidClusterer(radius_m=cfg.dedup_dist)

    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    telemetry_file = os.path.join(args.flight_dir, "telemetry.jsonl")

    fmap = None
    wp_file = f"{wp_dir}/survivors_{session}.waypoints"
    waypoints = []

    with open(telemetry_file, "r") as f:
        for line in f:
            rec = json.loads(line)

            img_path = os.path.join(args.flight_dir, rec["image"])
            img = cv2.imread(img_path)
            if img is None:
                continue

            drone = DroneState(
                frame_id=rec["frame_id"],
                timestamp=rec["timestamp"],
                lat=rec["lat"],
                lon=rec["lon"],
                alt_agl=rec["alt_agl"],
                heading_deg=rec["heading_deg"],
                pitch=rec["pitch"],
                roll=rec["roll"]
            )
            res = model.predict(img, conf=cfg.conf, imgsz=cfg.imgsz, verbose=False)[0]
            annotated = img.copy()

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
                north, east = rw[0]*t, rw[1]*t
                dist = math.hypot(north, east)

                if dist > cfg.max_distance:
                    continue

                lat, lon = meters_to_latlon(north, east, drone.lat, drone.lon)
                if not tracker.is_new(lat, lon, drone.timestamp):
                    continue
                clat, clon, is_new = clusterer.add(lat, lon)
                if fmap is None:
                    fmap = folium.Map(location=[clat, clon], zoom_start=18)
                if is_new:
                    idx = len(waypoints)
                    waypoints.append((idx, 0, 3, 16, 0, 0, 0, 0, clat, clon, RESCUE_ALTITUDE_M, 1))

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(
                    annotated,
                    f"{dist:.1f}m",
                    (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,0),
                    2
                )
                if is_new:
                    folium.CircleMarker([clat, clon], radius=6, color="red", fill=True).add_to(fmap)

            cv2.imwrite(f"{img_dir}/{session}_{drone.frame_id}.jpg", annotated)

    # Save waypoint file
    if waypoints:
        with open(wp_file, "w") as f:
            f.write("QGC WPL 110\n")
            for wp in waypoints:
                f.write("\t".join(map(str, wp)) + "\n")

    if fmap:
        fmap.save(f"{final_dir}/final_survivors_map_{session}.html")

    logger.info("Offline flight processing complete")

# =====================================================
if __name__ == "__main__":
    main()
