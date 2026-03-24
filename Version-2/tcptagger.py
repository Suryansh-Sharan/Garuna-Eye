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
MAX_CLUSTERS = 10   # safety cap
MIN_FRAMES_SEEN = 3  # temporal confirmation threshold

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

        self.cluster_radius = cfg["deduplication"]["min_distance_meters"]

        self.pitch_offset = cfg["gimbal"]["pitch_offset_deg"]

        self.tcp_host = cfg["network"]["tcp_host"]
        self.tcp_port = cfg["network"]["tcp_port"]

        self.max_distance = cfg["validation"]["max_geotag_distance"]

        self.map_tiles = cfg["advanced"]["map_tile_server"]
        self.map_zoom = cfg["advanced"]["map_zoom_level"]

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
# FRAME STABILIZATION (ORB + RANSAC HOMOGRAPHY)
# =====================================================
def compute_homography(img, ref_frame, mask=None):
    """
    Compute a homography that warps `img` onto `ref_frame` using ORB features
    and RANSAC. Falls back to identity if matching fails.
    
    Args:
        img: Current frame
        ref_frame: Reference frame
        mask: Optional binary mask (255 = use, 0 = ignore) for img
    """
    # Convert to grayscale
    gray1 = cv2.cvtColor(ref_frame, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # ORB detector
    orb = cv2.ORB_create(1000)
    kps1, des1 = orb.detectAndCompute(gray1, None)
    kps2, des2 = orb.detectAndCompute(gray2, mask)

    if des1 is None or des2 is None or len(kps1) < 4 or len(kps2) < 4:
        return np.eye(3, dtype=np.float32)

    # Brute-Force matcher with Hamming distance
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    if len(matches) < 8:
        return np.eye(3, dtype=np.float32)

    # Sort matches by distance (best first)
    matches = sorted(matches, key=lambda m: m.distance)[:200]

    pts1 = np.float32([kps1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    pts2 = np.float32([kps2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

    H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
    if H is None:
        return np.eye(3, dtype=np.float32)

    return H.astype(np.float32)

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

class SingleSurvivorWaypointWriter:
    def __init__(self, home_lat, home_lon, altitude):
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.altitude = altitude
        self.counter = 1

    def save_survivor(self, lat, lon):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"survivor_{self.counter:03d}_{timestamp}.waypoints"
        path = os.path.join(WAYPOINT_DIR, filename)

        with open(path, "w") as f:
            f.write("QGC WPL 110\n")

            # HOME
            f.write(
                f"0\t1\t0\t16\t0\t0\t0\t0\t"
                f"{self.home_lat}\t{self.home_lon}\t{self.altitude}\t1\n"
            )

            # SURVIVOR
            f.write(
                f"1\t0\t3\t16\t0\t0\t0\t0\t"
                f"{lat}\t{lon}\t{self.altitude}\t1\n"
            )

            # RTL
            f.write(
                "2\t0\t3\t20\t0\t0\t0\t0\t0\t0\t0\t1\n"
            )

        print(f"✅ Survivor mission saved: {filename}")
        self.counter += 1

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

        drone = DroneState(
            frame_id=meta["frame_id"],
            timestamp=meta["timestamp"],
            lat=meta["lat"],
            lon=meta["lon"],
            alt_agl=meta["alt_agl"],
            heading_deg=meta["heading_deg"],
            pitch=meta["pitch"],
            roll=meta["roll"]
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
# TEMPORAL + SPATIAL CLUSTERER
# =====================================================
class CentroidClusterer:
    def __init__(self, base_radius):
        self.base_radius = base_radius
        # Each cluster: [lat, lon, frames_seen, emitted]
        self.clusters = []

    def add(self, lat, lon, altitude):
        """
        Temporal + spatial clustering:
        - Merge if distance(prev_point, new_point) < max(altitude * 0.7, 5)
        - Otherwise, start a new survivor candidate.
        - Only emit survivor once when frames_seen == MIN_FRAMES_SEEN
        """
        radius = max(altitude * 0.7, 5.0)

        for c in self.clusters:
            if haversine(lat, lon, c[0], c[1]) < radius:
                # Merge into existing cluster
                c[2] += 1
                c[0] += (lat - c[0]) / c[2]
                c[1] += (lon - c[1]) / c[2]
                # Emit only once when frames_seen == MIN_FRAMES_SEEN
                if c[2] == MIN_FRAMES_SEEN and not c[3]:
                    c[3] = True  # Mark as emitted
                    return c[0], c[1], True
                return c[0], c[1], False

        if len(self.clusters) >= MAX_CLUSTERS:
            return lat, lon, False

        # New survivor candidate, seen for the first time
        self.clusters.append([lat, lon, 1, False])  # [lat, lon, frames_seen, emitted]
        return lat, lon, False

# =====================================================
# MAIN
# =====================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument(
        "--map-tile",
        help="Path to a map tile image (e.g., png/jpg) for anchoring the reference frame"
    )
    args = ap.parse_args()

    cfg = Config(args.config)
    model = YOLO(cfg.model_path)
    rx = TCPReceiver(cfg.tcp_host, cfg.tcp_port)
    clusterer = CentroidClusterer(cfg.cluster_radius)

    fmap = None
    wp_writer = None
    ref_frame = None
    map_tile = cv2.imread(args.map_tile) if args.map_tile else None
    H_ref_to_map = None
    map_h = map_w = None
    lat0 = lon0 = None

    while True:
        img, drone = rx.receive()
        if img is None:
            continue

        # HARD reject non-nadir frames
        if abs(drone.roll) > 5 or abs(drone.pitch + 90) > 5:
            continue

        # =====================================================
        # STEP 2: FRAME STABILIZATION (MANDATORY)
        # =====================================================
        h, w = img.shape[:2]

        # First frame becomes reference (frozen, never updated)
        if ref_frame is None:
            ref_frame = img.copy()
            if map_tile is not None:
                map_h, map_w = map_tile.shape[:2]
                lat0, lon0 = drone.lat, drone.lon
            continue

        # Run YOLO first to get detections for masking (exclude moving objects)
        res_temp = model.predict(img, conf=cfg.conf, imgsz=cfg.imgsz, verbose=False)[0]
        
        # Create mask excluding YOLO bounding boxes (+padding) - mask all detections
        mask = np.ones((h, w), dtype=np.uint8) * 255
        padding = 10
        for b in res_temp.boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0])
            x1 = max(0, x1 - padding)
            y1 = max(0, y1 - padding)
            x2 = min(w, x2 + padding)
            y2 = min(h, y2 + padding)
            mask[y1:y2, x1:x2] = 0

        # Compute stabilization homography with masked ORB
        H = compute_homography(img, ref_frame, mask)  # ORB + RANSAC with masking
        img = cv2.warpPerspective(img, H, (w, h))

        # =====================================================
        # STEP 3: MAP ANCHORING (MAGIC)
        # Compute homography from reference frame to map tile ONCE
        # First compute map → ref, then invert to get ref → map
        # =====================================================
        if map_tile is not None and H_ref_to_map is None:
            H_map_to_ref = compute_homography(map_tile, ref_frame)
            H_ref_to_map = np.linalg.inv(H_map_to_ref)

        res = model.predict(img, conf=cfg.conf, imgsz=cfg.imgsz, verbose=False)[0]
        annotated = img.copy()

        for b in res.boxes:
            cls = res.names[int(b.cls[0])].lower()
            if cls not in cfg.human_classes:
                continue

            x1, y1, x2, y2 = map(int, b.xyxy[0])
            u, v = (x1 + x2) / 2, y2

            # =====================================================
            # STEP 3 & 4: Project detection to map pixels, then to Lat/Lon
            # =====================================================
            if H_ref_to_map is None or map_tile is None or map_h is None or map_w is None or lat0 is None or lon0 is None:
                # Cannot anchor without a map tile and homography
                continue

            p_img = np.array([u, v, 1.0], dtype=np.float32)
            p_map = H_ref_to_map @ p_img
            if p_map[2] == 0:
                continue
            p_map = p_map / p_map[2]

            # Map pixel -> Lat/Lon (paper formula)
            meters_per_pixel = 156543.03392 / (2 ** cfg.map_zoom)
            dlat = (p_map[1] - map_h / 2) * meters_per_pixel / R_EARTH
            dlon = (p_map[0] - map_w / 2) * meters_per_pixel / (R_EARTH * math.cos(math.radians(lat0)))

            lat = lat0 + math.degrees(dlat)
            lon = lon0 + math.degrees(dlon)

            dist = haversine(drone.lat, drone.lon, lat, lon)
            if dist > cfg.max_distance:
                continue

            clat, clon, is_confirmed = clusterer.add(lat, lon, drone.alt_agl)

            # Require min_frames_seen >= MIN_FRAMES_SEEN before we create a survivor
            if not is_confirmed:
                continue

            if wp_writer is None:
                wp_writer = SingleSurvivorWaypointWriter(
                    home_lat=drone.lat,
                    home_lon=drone.lon,
                    altitude=RESCUE_ALTITUDE_M
                )

            if fmap is None:
                fmap = folium.Map(
                    location=[drone.lat, drone.lon],
                    zoom_start=cfg.map_zoom,
                    tiles=cfg.map_tiles
                )

            wp_writer.save_survivor(clat, clon)
            folium.CircleMarker(
                [clat, clon],
                radius=6,
                color="red",
                fill=True
            ).add_to(fmap)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(annotated, f"{dist:.1f} m",
                        (x1, y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        cv2.imwrite(f"{IMAGE_DIR}/{SESSION_ID}_{drone.frame_id}.jpg", annotated)
        if fmap:
            fmap.save(f"{MAP_DIR}/survivors_map_{SESSION_ID}.html")

# =====================================================
if __name__ == "__main__":
    main()
