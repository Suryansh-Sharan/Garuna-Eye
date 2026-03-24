#!/usr/bin/env python3
"""
Backend Skeleton: Race-Condition-Safe Drone Ground Control System

This skeleton implements the architecture design with:
- Producer-consumer queues
- Single processing owner
- FastAPI (REST + WebSocket)
- Thread-safe survivor state management
- Real TCP receiver (Raspberry Pi → frame_queue)
- YOLO inference with spatial clustering
- Survivor confirmation (MIN_FRAMES_SEEN threshold)

TODO:
- Waypoint file generation (on confirmation)
- Frame stabilization (ORB + RANSAC)
"""

import asyncio
import json
import logging
import math
import os
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

# =====================================================
# LOGGING
# =====================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# =====================================================
# CONSTANTS
# =====================================================
MIN_FRAMES_SEEN = 3  # Temporal confirmation threshold
FRAME_QUEUE_SIZE = 10
WEBSOCKET_EVENT_QUEUE_SIZE = 100
DISPATCH_QUEUE_SIZE = 50
R_EARTH = 6378137.0  # Earth radius in meters
RESCUE_ALTITUDE_M = 10.0  # Altitude for waypoint missions

# TCP Configuration
TCP_HOST = "0.0.0.0"  # Listen on all interfaces
TCP_PORT = 7000

# YOLO Configuration (loaded from config.yaml)
YOLO_CONFIG_PATH = "config.yaml"

# Output directories
BASE_DIR = "outputs"
WAYPOINT_DIR = os.path.join(BASE_DIR, "waypoints")
os.makedirs(WAYPOINT_DIR, exist_ok=True)

# =====================================================
# DATA MODELS
# =====================================================

@dataclass
class Survivor:
    """Survivor data model with sequential ID format."""
    id: str                    # Sequential ID: "surv_001", "surv_002", ...
    lat: float                 # Confirmed latitude
    lon: float                 # Confirmed longitude
    status: str                # CANDIDATE | CONFIRMED | DISPATCHED
    frames_seen: int           # Temporal confirmation counter
    first_seen: float          # Unix timestamp of first detection
    confirmed_at: Optional[float] = None  # Unix timestamp when confirmed
    dispatched_at: Optional[float] = None  # Unix timestamp when dispatched
    waypoint_file: Optional[str] = None    # Path to waypoint file
    detection_history: List[Tuple[float, float, float]] = field(default_factory=list)  # [(lat, lon, timestamp), ...]
    
    def to_dict(self) -> dict:
        """Create immutable snapshot for API responses."""
        return {
            "id": self.id,
            "lat": self.lat,
            "lon": self.lon,
            "status": self.status,
            "frames_seen": self.frames_seen,
            "first_seen": self.first_seen,
            "confirmed_at": self.confirmed_at,
            "dispatched_at": self.dispatched_at,
            "waypoint_file": self.waypoint_file
        }


@dataclass
class DroneState:
    """Telemetry data model (stub for now)."""
    frame_id: int
    timestamp: float
    lat: float
    lon: float
    alt_agl: float
    heading_deg: float
    pitch: float
    roll: float


# =====================================================
# SURVIVOR MANAGER (Lock-Protected State)
# =====================================================

class SurvivorManager:
    """
    Thread-safe survivor state manager.
    
    Single writer: Processing thread (all mutations)
    Multiple readers: FastAPI thread (read-only snapshots)
    """
    
    def __init__(self):
        self._lock = threading.Lock()
        self._registry: Dict[str, Survivor] = {}
        self._next_id_counter = 1  # Atomic counter for sequential IDs
    
    def create_candidate(self, lat: float, lon: float, timestamp: float) -> Survivor:
        """
        Create new survivor candidate.
        
        CALLED BY: Processing thread only (lock-protected)
        
        CRITICAL: ID generation and registry insertion are atomic (same lock block).
        This ensures no FastAPI reads can interleave between ID generation and insertion.
        """
        with self._lock:
            # Generate ID and increment counter atomically
            survivor_id = f"surv_{self._next_id_counter:03d}"
            self._next_id_counter += 1
            
            # Create survivor and insert into registry atomically
            survivor = Survivor(
                id=survivor_id,
                lat=lat,
                lon=lon,
                status="CANDIDATE",
                frames_seen=1,
                first_seen=timestamp,
                detection_history=[(lat, lon, timestamp)]
            )
            
            self._registry[survivor_id] = survivor
        
        logger.info(f"Created survivor candidate: {survivor_id} at ({lat:.6f}, {lon:.6f})")
        return survivor
    
    def update_survivor(self, survivor_id: str, lat: float, lon: float, timestamp: float) -> Tuple[Optional[Survivor], bool]:
        """
        Update existing survivor (increment frames_seen, update centroid).
        
        CALLED BY: Processing thread only (lock-protected)
        
        Returns:
            (survivor, was_confirmed): survivor object and whether it was just confirmed
        """
        with self._lock:
            if survivor_id not in self._registry:
                return None, False
            
            survivor = self._registry[survivor_id]
            was_candidate = survivor.status == "CANDIDATE"
            old_frames_seen = survivor.frames_seen
            
            survivor.frames_seen += 1
            # Update centroid (running average)
            survivor.lat = (survivor.lat * old_frames_seen + lat) / survivor.frames_seen
            survivor.lon = (survivor.lon * old_frames_seen + lon) / survivor.frames_seen
            survivor.detection_history.append((lat, lon, timestamp))
            
            # Check if this update triggered confirmation
            was_confirmed = False
            if was_candidate and survivor.frames_seen >= MIN_FRAMES_SEEN and old_frames_seen < MIN_FRAMES_SEEN:
                survivor.status = "CONFIRMED"
                survivor.confirmed_at = timestamp
                was_confirmed = True
            
            return survivor, was_confirmed
    
    def set_waypoint_file(self, survivor_id: str, waypoint_file: str) -> bool:
        """
        Set waypoint file path for a survivor.
        
        CALLED BY: Processing thread only (lock-protected)
        """
        with self._lock:
            if survivor_id not in self._registry:
                return False
            
            survivor = self._registry[survivor_id]
            survivor.waypoint_file = waypoint_file
            logger.info(f"Set waypoint file for survivor {survivor_id}: {waypoint_file}")
            return True
    
    def confirm_survivor(self, survivor_id: str, waypoint_file: str, timestamp: float) -> bool:
        """
        Transition survivor from CANDIDATE → CONFIRMED.
        
        CALLED BY: Processing thread only (lock-protected)
        """
        with self._lock:
            if survivor_id not in self._registry:
                return False
            
            survivor = self._registry[survivor_id]
            if survivor.status != "CANDIDATE":
                return False
            
            survivor.status = "CONFIRMED"
            survivor.confirmed_at = timestamp
            survivor.waypoint_file = waypoint_file
            
            logger.info(f"Confirmed survivor: {survivor_id} (frames_seen={survivor.frames_seen})")
            return True
    
    def dispatch_survivor(self, survivor_id: str, timestamp: float) -> bool:
        """
        Transition survivor from CONFIRMED → DISPATCHED.
        
        CALLED BY: Processing thread only (lock-protected)
        """
        with self._lock:
            if survivor_id not in self._registry:
                return False
            
            survivor = self._registry[survivor_id]
            if survivor.status != "CONFIRMED":
                return False
            
            survivor.status = "DISPATCHED"
            survivor.dispatched_at = timestamp
            
            logger.info(f"Dispatched survivor: {survivor_id}")
            return True
    
    def get_survivor(self, survivor_id: str) -> Optional[Survivor]:
        """
        Get single survivor (read-only snapshot).
        
        CALLED BY: FastAPI thread (lock-protected read)
        """
        with self._lock:
            survivor = self._registry.get(survivor_id)
            if survivor is None:
                return None
            # Return a copy to ensure immutability
            return Survivor(
                id=survivor.id,
                lat=survivor.lat,
                lon=survivor.lon,
                status=survivor.status,
                frames_seen=survivor.frames_seen,
                first_seen=survivor.first_seen,
                confirmed_at=survivor.confirmed_at,
                dispatched_at=survivor.dispatched_at,
                waypoint_file=survivor.waypoint_file,
                detection_history=survivor.detection_history.copy()
            )
    
    def get_all_survivors(self) -> Dict[str, Survivor]:
        """
        Get all survivors as immutable snapshot.
        
        CALLED BY: FastAPI thread (lock-protected read)
        """
        with self._lock:
            # Create immutable snapshot
            snapshot = {}
            for survivor_id, survivor in self._registry.items():
                snapshot[survivor_id] = Survivor(
                    id=survivor.id,
                    lat=survivor.lat,
                    lon=survivor.lon,
                    status=survivor.status,
                    frames_seen=survivor.frames_seen,
                    first_seen=survivor.first_seen,
                    confirmed_at=survivor.confirmed_at,
                    dispatched_at=survivor.dispatched_at,
                    waypoint_file=survivor.waypoint_file,
                    detection_history=survivor.detection_history.copy()
                )
            return snapshot


# =====================================================
# QUEUES (Thread-Safe Communication)
# =====================================================

# Frame queue: TCP Receiver → Processing Loop
frame_queue: queue.Queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)

# WebSocket event queue: Processing Loop → FastAPI (thread-safe, drops oldest when full)
websocket_event_queue: queue.Queue = queue.Queue(maxsize=WEBSOCKET_EVENT_QUEUE_SIZE)

# Dispatch queue: FastAPI → Processing Loop
dispatch_queue: queue.Queue = queue.Queue(maxsize=DISPATCH_QUEUE_SIZE)

# Shutdown flag
shutdown_flag = threading.Event()


# =====================================================
# THREAD 1: TCP RECEIVER
# =====================================================

def recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
    """
    Receive exactly n bytes from socket.
    
    Returns None if connection closed, otherwise returns bytes.
    Uses blocking recv() - OK because this runs in isolated thread.
    """
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None  # Connection closed
        buf += chunk
    return buf


def tcp_receiver_thread():
    """
    TCP Receiver Thread.
    
    ALLOWED:
    - Receive TCP packets
    - Decode frames and telemetry
    - Push to frame_queue
    
    FORBIDDEN:
    - Never run YOLO
    - Never access survivor state
    - Never write waypoint files
    - Never block indefinitely on queue operations
    """
    logger.info(f"TCP Receiver thread started, listening on {TCP_HOST}:{TCP_PORT}")
    
    server_sock = None
    client_sock = None
    
    while not shutdown_flag.is_set():
        try:
            # Create and bind server socket
            if server_sock is None:
                server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_sock.bind((TCP_HOST, TCP_PORT))
                server_sock.listen(1)
                server_sock.settimeout(1.0)  # Allow periodic shutdown_flag check
                logger.info(f"TCP server listening on {TCP_HOST}:{TCP_PORT}")
            
            # Accept connection (with timeout to check shutdown_flag)
            if client_sock is None:
                try:
                    client_sock, addr = server_sock.accept()
                    logger.info(f"TCP client connected from {addr}")
                except socket.timeout:
                    continue  # Check shutdown_flag and retry
                except OSError as e:
                    if shutdown_flag.is_set():
                        break
                    logger.error(f"Socket accept error: {e}")
                    time.sleep(1.0)
                    continue
            
            # Receive packet size header (4 bytes, big-endian)
            size_header = recv_exact(client_sock, 4)
            if size_header is None:
                logger.warning("TCP client disconnected (during header read)")
                client_sock.close()
                client_sock = None
                continue
            
            # Unpack payload size (big-endian unsigned int)
            payload_size = struct.unpack(">I", size_header)[0]
            
            # Validate payload size (sanity check)
            if payload_size > 10 * 1024 * 1024:  # 10MB max
                logger.error(f"Invalid payload size: {payload_size} bytes (too large)")
                client_sock.close()
                client_sock = None
                continue
            
            # Receive payload
            payload = recv_exact(client_sock, payload_size)
            if payload is None:
                logger.warning("TCP client disconnected (during payload read)")
                client_sock.close()
                client_sock = None
                continue
            
            # Parse JSON metadata (ends at first '}')
            meta_end = payload.find(b"}") + 1
            if meta_end == 0:
                logger.error("Invalid packet: JSON metadata not found")
                continue
            
            try:
                meta_json = payload[:meta_end].decode("utf-8")
                meta = json.loads(meta_json)
            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                logger.error(f"Failed to decode JSON metadata: {e}")
                continue
            
            # Extract JPEG image bytes
            jpeg_bytes = payload[meta_end:]
            
            # Decode JPEG image
            try:
                frame = cv2.imdecode(
                    np.frombuffer(jpeg_bytes, np.uint8),
                    cv2.IMREAD_COLOR
                )
                if frame is None:
                    logger.error("Failed to decode JPEG image")
                    continue
            except Exception as e:
                logger.error(f"JPEG decode error: {e}")
                continue
            
            # Construct DroneState from metadata
            try:
                telemetry = DroneState(
                    frame_id=meta["frame_id"],
                    timestamp=meta["timestamp"],
                    lat=meta["lat"],
                    lon=meta["lon"],
                    alt_agl=meta["alt_agl"],
                    heading_deg=meta["heading_deg"],
                    pitch=meta["pitch"],
                    roll=meta["roll"]
                )
            except KeyError as e:
                logger.error(f"Missing field in metadata: {e}")
                continue
            
            # Push to queue (non-blocking, drop if full)
            try:
                frame_queue.put_nowait((frame, telemetry))
                logger.debug(f"Pushed frame {telemetry.frame_id} to queue")
            except queue.Full:
                logger.warning(f"Frame queue full - dropping frame {telemetry.frame_id}")
                # Drop frame (design requirement: stability > FPS)
        
        except socket.error as e:
            logger.error(f"TCP socket error: {e}")
            if client_sock:
                try:
                    client_sock.close()
                except:
                    pass
                client_sock = None
            time.sleep(1.0)
        
        except Exception as e:
            logger.error(f"TCP receiver error: {e}", exc_info=True)
            if client_sock:
                try:
                    client_sock.close()
                except:
                    pass
                client_sock = None
            time.sleep(1.0)
    
    # Cleanup
    if client_sock:
        try:
            client_sock.close()
        except:
            pass
    if server_sock:
        try:
            server_sock.close()
        except:
            pass
    
    logger.info("TCP Receiver thread stopped")


# =====================================================
# GEOMETRY & CLUSTERING
# =====================================================

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate haversine distance between two lat/lon points in meters."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return R_EARTH * 2 * math.asin(math.sqrt(a))


def pixel_to_latlon_simple(
    u: float, v: float, 
    img_w: int, img_h: int,
    drone_lat: float, drone_lon: float,
    alt_agl: float, heading_deg: float,
    hfov_deg: float, vfov_deg: float
) -> Tuple[float, float]:
    """
    Simplified geolocation: project pixel to ground lat/lon.
    
    Uses camera FOV and altitude to estimate ground position.
    This is a simplified approximation for clustering purposes.
    """
    # Normalize pixel coordinates to [-1, 1]
    x_norm = (u - img_w / 2) / (img_w / 2)
    y_norm = (v - img_h / 2) / (img_h / 2)
    
    # Calculate ground distance from drone (simplified: assumes nadir view)
    # Horizontal distance = altitude * tan(hfov/2) * x_norm
    # Vertical distance = altitude * tan(vfov/2) * y_norm
    hfov_rad = math.radians(hfov_deg / 2)
    vfov_rad = math.radians(vfov_deg / 2)
    
    # Ground distance in meters (simplified projection)
    east_m = alt_agl * math.tan(hfov_rad) * x_norm
    north_m = alt_agl * math.tan(vfov_rad) * y_norm
    
    # Rotate by heading
    heading_rad = math.radians(heading_deg)
    rotated_east = east_m * math.cos(heading_rad) - north_m * math.sin(heading_rad)
    rotated_north = east_m * math.sin(heading_rad) + north_m * math.cos(heading_rad)
    
    # Convert meters to lat/lon
    dlat = rotated_north / R_EARTH
    dlon = rotated_east / (R_EARTH * math.cos(math.radians(drone_lat)))
    
    lat = drone_lat + math.degrees(dlat)
    lon = drone_lon + math.degrees(dlon)
    
    return lat, lon


# =====================================================
# WAYPOINT FILE GENERATION
# =====================================================

def generate_waypoint_file(
    survivor_id: str,
    survivor_lat: float,
    survivor_lon: float,
    home_lat: float,
    home_lon: float,
    altitude: float = RESCUE_ALTITUDE_M
) -> str:
    """
    Generate QGC WPL 110 format waypoint file for a survivor.
    
    Format (Mission Planner compatible):
    - Index 0: HOME waypoint (current=1, frame=0, command=16)
    - Index 1: SURVIVOR waypoint (current=0, frame=3, command=16)
    
    QGC WPL 110 format is compatible with:
    - QGroundControl
    - Mission Planner
    - MAVLink-compatible autopilots
    
    CALLED BY: Processing thread only (atomic operation)
    
    Returns:
        Path to generated waypoint file
    """
    # Generate filename: survivor_<id>.waypoints
    # Extract numeric ID from survivor_id (e.g., "surv_001" -> "001")
    id_num = survivor_id.replace("surv_", "")
    filename = f"survivor_{id_num}.waypoints"
    filepath = os.path.join(WAYPOINT_DIR, filename)
    
    # Write QGC WPL 110 format
    with open(filepath, "w") as f:
        f.write("QGC WPL 110\n")
        
        # HOME waypoint (index 0)
        # Format: seq, current, frame, command, param1, param2, param3, param4, lat, lon, alt, autocontinue
        # command 16 = NAV_WAYPOINT, current=1 means this is the home position
        f.write(
            f"0\t1\t0\t16\t0\t0\t0\t0\t"
            f"{home_lat:.7f}\t{home_lon:.7f}\t{altitude:.1f}\t1\n"
        )
        
        # SURVIVOR waypoint (index 1)
        # command 16 = NAV_WAYPOINT, current=0 means this is a mission waypoint
        f.write(
            f"1\t0\t3\t16\t0\t0\t0\t0\t"
            f"{survivor_lat:.7f}\t{survivor_lon:.7f}\t{altitude:.1f}\t1\n"
        )
    
    logger.info(f"Generated waypoint file: {filepath}")
    return filepath


class SpatialClusterer:
    """
    Spatial clustering for survivor detections.
    
    Maps detections to existing survivors or creates new ones.
    Uses haversine distance with altitude-based radius.
    """
    
    def __init__(self, survivor_manager: SurvivorManager):
        self.survivor_manager = survivor_manager
        # Map: (lat, lon) cluster key -> survivor_id
        # We use a simple approach: check all existing survivors
        # and find the closest one within radius
    
    def process_detection(
        self, 
        lat: float, 
        lon: float, 
        altitude: float, 
        timestamp: float
    ) -> Tuple[Optional[str], bool]:
        """
        Process a detection: cluster spatially and create/update survivor.
        
        Returns:
            (survivor_id, was_confirmed): survivor_id if processed (None if skipped), 
                                         was_confirmed indicates if just confirmed
        """
        # Calculate clustering radius (altitude-based)
        radius = max(altitude * 0.7, 5.0)  # meters
        
        # Get all existing survivors (snapshot)
        # Note: We need to check all survivors, but we can't hold lock during clustering
        # So we'll do a two-phase approach:
        # 1. Find closest survivor (read-only, with lock)
        # 2. Update or create (write, with lock)
        
        with self.survivor_manager._lock:
            survivors = self.survivor_manager._registry.copy()
        
        # Find closest survivor within radius
        closest_survivor_id = None
        min_distance = float('inf')
        
        for survivor_id, survivor in survivors.items():
            if survivor.status == "DISPATCHED":
                continue  # Skip dispatched survivors
            
            distance = haversine(lat, lon, survivor.lat, survivor.lon)
            if distance < radius and distance < min_distance:
                min_distance = distance
                closest_survivor_id = survivor_id
        
        # Update existing survivor or create new one
        if closest_survivor_id is not None:
            # Update existing survivor
            survivor, was_confirmed = self.survivor_manager.update_survivor(
                closest_survivor_id, lat, lon, timestamp
            )
            if survivor is not None:
                return closest_survivor_id, was_confirmed
        else:
            # Create new candidate
            survivor = self.survivor_manager.create_candidate(lat, lon, timestamp)
            
            # Emit candidate event
            emit_websocket_event({
                "type": "SURVIVOR_CANDIDATE",
                "timestamp": timestamp,
                "data": {
                    "survivor_id": survivor.id,
                    "lat": survivor.lat,
                    "lon": survivor.lon,
                    "frames_seen": survivor.frames_seen,
                    "first_seen": survivor.first_seen
                }
            })
            
            return survivor.id, False
        
        return None, False


# =====================================================
# THREAD 2: PROCESSING LOOP
# =====================================================

def processing_loop_thread(survivor_manager: SurvivorManager):
    """
    Processing Loop Thread.
    
    SINGLE OWNER of all state mutations:
    - Consumes frames from queue
    - Runs YOLO inference
    - Updates survivor state (lock-protected)
    - Emits WebSocket events
    
    FORBIDDEN:
    - Never read from TCP socket directly
    - Never serve HTTP requests
    - Never mutate state without lock
    """
    logger.info("Processing loop thread started")
    
    # Load YOLO configuration
    try:
        with open(YOLO_CONFIG_PATH, "r") as f:
            config = yaml.safe_load(f)
        
        model_path = config["yolo"]["model_path"]
        confidence_threshold = config["yolo"]["confidence_threshold"]
        imgsz = config["yolo"]["imgsz"]
        human_classes = [cls.lower() for cls in config["yolo"]["human_classes"]]
        hfov_deg = config["camera"]["hfov_deg"]
        vfov_deg = config["camera"]["vfov_deg"]
        max_distance = config["validation"]["max_geotag_distance"]
        
        # Load YOLO model
        logger.info(f"Loading YOLO model from {model_path}")
        yolo_model = YOLO(model_path)
        logger.info("YOLO model loaded successfully")
        
    except Exception as e:
        logger.error(f"Failed to load YOLO configuration or model: {e}", exc_info=True)
        logger.error("Processing loop will exit")
        return
    
    # Initialize spatial clusterer
    clusterer = SpatialClusterer(survivor_manager)
    
    while not shutdown_flag.is_set():
        try:
            # Process dispatch queue first (non-blocking)
            process_dispatch_queue(survivor_manager)
            
            # Consume frame from queue (with timeout)
            try:
                frame, telemetry = frame_queue.get(timeout=0.1)
            except queue.Empty:
                # No frame available, continue loop
                continue
            
            # Frame validation: reject non-nadir frames
            if abs(telemetry.roll) > 5 or abs(telemetry.pitch + 90) > 5:
                logger.debug(f"Rejected frame {telemetry.frame_id}: non-nadir (pitch={telemetry.pitch:.1f}, roll={telemetry.roll:.1f})")
                continue
            
            # Validate frame is not None
            if frame is None:
                logger.warning(f"Frame {telemetry.frame_id} is None, skipping")
                continue
            
            # Run YOLO inference
            try:
                results = yolo_model.predict(
                    frame,
                    conf=confidence_threshold,
                    imgsz=imgsz,
                    verbose=False
                )
                result = results[0]  # Get first (and only) result
            except Exception as e:
                logger.error(f"YOLO inference error on frame {telemetry.frame_id}: {e}")
                continue
            
            # Process detections
            img_h, img_w = frame.shape[:2]
            detections_processed = 0
            
            for box in result.boxes:
                # Check if detection is a human class
                cls_name = result.names[int(box.cls[0])].lower()
                if cls_name not in human_classes:
                    continue
                
                # Extract bounding box center (bottom center for ground projection)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                u = (x1 + x2) / 2.0  # Center X
                v = float(y2)  # Bottom Y (ground contact point)
                
                # Compute geolocation (simplified)
                try:
                    lat, lon = pixel_to_latlon_simple(
                        u, v, img_w, img_h,
                        telemetry.lat, telemetry.lon,
                        telemetry.alt_agl, telemetry.heading_deg,
                        hfov_deg, vfov_deg
                    )
                except Exception as e:
                    logger.error(f"Geolocation error: {e}")
                    continue
                
                # Validate distance from drone
                distance = haversine(telemetry.lat, telemetry.lon, lat, lon)
                if distance > max_distance:
                    logger.debug(f"Detection too far: {distance:.1f}m > {max_distance}m")
                    continue
                
                # Cluster spatially and create/update survivor
                survivor_id, was_confirmed = clusterer.process_detection(
                    lat, lon, telemetry.alt_agl, telemetry.timestamp
                )
                
                if survivor_id:
                    detections_processed += 1
                    logger.debug(f"Processed detection for survivor {survivor_id} (frame {telemetry.frame_id})")
                    
                    # If survivor was just confirmed, generate waypoint file
                    if was_confirmed:
                        # Get survivor details for waypoint generation
                        survivor = survivor_manager.get_survivor(survivor_id)
                        if survivor:
                            # Generate waypoint file atomically
                            waypoint_file = generate_waypoint_file(
                                survivor_id=survivor_id,
                                survivor_lat=survivor.lat,
                                survivor_lon=survivor.lon,
                                home_lat=telemetry.lat,  # Drone position when confirmed
                                home_lon=telemetry.lon,
                                altitude=RESCUE_ALTITUDE_M
                            )
                            
                            # Attach waypoint file to survivor
                            survivor_manager.set_waypoint_file(survivor_id, waypoint_file)
                            
                            # Emit confirmation event with waypoint file path
                            emit_websocket_event({
                                "type": "SURVIVOR_CONFIRMED",
                                "timestamp": telemetry.timestamp,
                                "data": {
                                    "survivor_id": survivor_id,
                                    "lat": survivor.lat,
                                    "lon": survivor.lon,
                                    "frames_seen": survivor.frames_seen,
                                    "confirmed_at": survivor.confirmed_at,
                                    "waypoint_file": waypoint_file
                                }
                            })
                            
                            logger.info(f"Survivor {survivor_id} confirmed with waypoint file: {waypoint_file}")
            
            if detections_processed > 0:
                logger.info(f"Frame {telemetry.frame_id}: processed {detections_processed} detection(s)")
        
        except Exception as e:
            logger.error(f"Processing loop error: {e}", exc_info=True)
            time.sleep(0.1)
    
    logger.info("Processing loop thread stopped")


def process_dispatch_queue(survivor_manager: SurvivorManager):
    """
    Process dispatch requests from queue.
    
    Dispatch flow:
    1. Mark survivor status as DISPATCHED (atomic, lock-protected)
    2. Waypoint file remains available for Mission Planner upload
    3. Emit SURVIVOR_DISPATCHED event with waypoint file path
    
    CALLED BY: Processing thread only (lock-protected)
    """
    try:
        survivor_id = dispatch_queue.get_nowait()
        
        # Get survivor snapshot to retrieve waypoint file path
        survivor = survivor_manager.get_survivor(survivor_id)
        if survivor is None:
            logger.warning(f"Cannot dispatch: survivor {survivor_id} not found")
            return
        
        # Acquire lock and mutate state (atomic operation)
        timestamp = time.time()
        success = survivor_manager.dispatch_survivor(survivor_id, timestamp)
        
        if success:
            # Emit SURVIVOR_DISPATCHED event with waypoint file path
            # Waypoint file is ready for Mission Planner upload
            emit_websocket_event({
                "type": "SURVIVOR_DISPATCHED",
                "timestamp": timestamp,
                "data": {
                    "survivor_id": survivor_id,
                    "dispatched_at": timestamp,
                    "waypoint_file": survivor.waypoint_file  # Include for Mission Planner upload
                }
            })
            logger.info(f"Dispatched survivor {survivor_id} - waypoint file ready: {survivor.waypoint_file}")
        else:
            logger.warning(f"Failed to dispatch survivor: {survivor_id} (may already be dispatched or not CONFIRMED)")
    
    except queue.Empty:
        pass  # No dispatch requests


def emit_websocket_event(event: dict):
    """
    Push event to WebSocket queue (thread-safe, drops oldest if full).
    
    CALLED BY: Processing thread only
    
    CRITICAL: Uses queue.Queue (thread-safe), not deque.
    When full, put_nowait() raises queue.Full - we drop the event.
    For FIFO + drop-oldest behavior, we'd need a custom queue wrapper,
    but for now dropping newest on overflow is acceptable.
    """
    try:
        websocket_event_queue.put_nowait(event)
        logger.debug(f"Emitted WebSocket event: {event['type']}")
    except queue.Full:
        logger.warning(f"WebSocket event queue full, dropping event: {event['type']}")
    except Exception as e:
        logger.error(f"Failed to emit WebSocket event: {e}")


# =====================================================
# FASTAPI APPLICATION
# =====================================================

app = FastAPI(title="Drone Ground Control System API")

# Global survivor manager instance
survivor_manager = SurvivorManager()

# WebSocket connections manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self._lock = threading.Lock()
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        with self._lock:
            self.active_connections.append(websocket)
        logger.info(f"WebSocket connected (total: {len(self.active_connections)})")
    
    def disconnect(self, websocket: WebSocket):
        with self._lock:
            if websocket in self.active_connections:
                self.active_connections.remove(websocket)
        logger.info(f"WebSocket disconnected (total: {len(self.active_connections)})")
    
    async def broadcast(self, message: dict):
        """Broadcast message to all connected WebSocket clients."""
        with self._lock:
            disconnected = []
            for connection in self.active_connections:
                try:
                    await connection.send_json(message)
                except Exception as e:
                    logger.warning(f"Failed to send WebSocket message: {e}")
                    disconnected.append(connection)
            
            # Remove disconnected clients
            for conn in disconnected:
                if conn in self.active_connections:
                    self.active_connections.remove(conn)

connection_manager = ConnectionManager()


# =====================================================
# REST API ENDPOINTS (Read-Only State Access)
# =====================================================

@app.get("/api/survivors")
async def get_survivors():
    """
    List all survivors (all statuses).
    
    READ-ONLY: Creates lock-protected snapshot
    """
    snapshot = survivor_manager.get_all_survivors()
    
    survivors_list = [surv.to_dict() for surv in snapshot.values()]
    
    return JSONResponse(content={
        "survivors": survivors_list,
        "count": len(survivors_list)
    })


@app.get("/api/survivors/{survivor_id}")
async def get_survivor(survivor_id: str):
    """
    Get single survivor by ID.
    
    READ-ONLY: Creates lock-protected snapshot
    """
    survivor = survivor_manager.get_survivor(survivor_id)
    
    if survivor is None:
        raise HTTPException(status_code=404, detail="Survivor not found")
    
    return JSONResponse(content=survivor.to_dict())


@app.get("/api/survivors/{survivor_id}/waypoint")
async def get_survivor_waypoint(survivor_id: str):
    """
    Download waypoint file for a survivor (Mission Planner compatible).
    
    Returns QGC WPL 110 format waypoint file that can be uploaded to:
    - Mission Planner
    - QGroundControl
    - MAVLink-compatible autopilots
    
    File format:
    - Index 0: HOME waypoint (drone position at confirmation)
    - Index 1: SURVIVOR waypoint (rescue location)
    
    READ-ONLY: Reads path (lock-protected), then reads file (no lock)
    """
    # Acquire lock to read waypoint_file path
    survivor = survivor_manager.get_survivor(survivor_id)
    
    if survivor is None:
        raise HTTPException(status_code=404, detail="Survivor not found")
    
    if survivor.waypoint_file is None:
        raise HTTPException(status_code=400, detail="Survivor has no waypoint file (status is CANDIDATE)")
    
    # Verify file exists before returning
    if not os.path.exists(survivor.waypoint_file):
        raise HTTPException(status_code=404, detail="Waypoint file not found")
    
    return FileResponse(
        path=survivor.waypoint_file,
        media_type="text/plain",
        filename=os.path.basename(survivor.waypoint_file),
        headers={"Content-Disposition": f'attachment; filename="{os.path.basename(survivor.waypoint_file)}"'}
    )


@app.post("/api/survivors/{survivor_id}/dispatch")
async def dispatch_survivor(survivor_id: str):
    """
    Dispatch a confirmed survivor for Mission Planner upload.
    
    Dispatch process:
    1. Validates survivor is CONFIRMED (read-only check)
    2. Queues dispatch request for processing thread
    3. Processing thread marks survivor as DISPATCHED (atomic)
    4. Waypoint file remains available via GET /api/survivors/{id}/waypoint
    5. SURVIVOR_DISPATCHED event emitted with waypoint file path
    
    Mission Planner compatibility:
    - Waypoint file is QGC WPL 110 format (Mission Planner compatible)
    - File can be uploaded directly to Mission Planner
    - No auto-fly: this endpoint only marks survivor as dispatched
    
    READ-ONLY: Validates existence (lock-protected), queues request
    State mutation happens in processing thread (atomic)
    """
    # Validate survivor exists and is in CONFIRMED status
    survivor = survivor_manager.get_survivor(survivor_id)
    
    if survivor is None:
        raise HTTPException(status_code=404, detail="Survivor not found")
    
    if survivor.status != "CONFIRMED":
        raise HTTPException(
            status_code=400 if survivor.status == "CANDIDATE" else 409,
            detail=f"Survivor is not in CONFIRMED status (current: {survivor.status})"
        )
    
    # Queue dispatch request (non-blocking)
    try:
        dispatch_queue.put_nowait(survivor_id)
        logger.info(f"Queued dispatch request for survivor: {survivor_id}")
    except queue.Full:
        raise HTTPException(status_code=503, detail="Dispatch queue full, please retry")
    
    return JSONResponse(content={
        "status": "queued",
        "survivor_id": survivor_id,
        "message": "Dispatch request queued for processing"
    }, status_code=202)


# =====================================================
# WEBSOCKET ENDPOINT
# =====================================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    WebSocket endpoint for live UI updates.
    
    Sends:
    - Initial state on connect
    - Real-time events are broadcast by background task
    """
    await connection_manager.connect(websocket)
    
    try:
        # Send initial state
        snapshot = survivor_manager.get_all_survivors()
        survivors_list = [surv.to_dict() for surv in snapshot.values()]
        
        await websocket.send_json({
            "type": "INITIAL_STATE",
            "survivors": survivors_list
        })
        
        # Keep connection alive and wait for disconnect
        # Events are broadcast by websocket_event_broadcaster background task
        while True:
            try:
                # Wait for client message or disconnect
                # Note: We don't use the received data - this is just to detect disconnects
                # In the future, could implement ping/pong here
                await websocket.receive_text()
            except WebSocketDisconnect:
                break
    
    except WebSocketDisconnect:
        pass
    finally:
        connection_manager.disconnect(websocket)


# =====================================================
# BACKGROUND TASK: WebSocket Event Broadcaster
# =====================================================

async def websocket_event_broadcaster():
    """
    Background task to broadcast WebSocket events.
    
    Polls websocket_event_queue (thread-safe queue) and broadcasts to all clients.
    """
    while not shutdown_flag.is_set():
        try:
            # Get event from queue (non-blocking, FIFO)
            event = websocket_event_queue.get_nowait()
            await connection_manager.broadcast(event)
        except queue.Empty:
            # No events available, sleep briefly
            await asyncio.sleep(0.1)
        except Exception as e:
            logger.error(f"WebSocket broadcaster error: {e}", exc_info=True)
            await asyncio.sleep(0.1)


# =====================================================
# MAIN ENTRY POINT
# =====================================================

def main():
    """Start all threads and FastAPI server."""
    logger.info("Starting backend skeleton...")
    
    # Start TCP receiver thread
    tcp_thread = threading.Thread(
        target=tcp_receiver_thread,
        name="TCPReceiver",
        daemon=True
    )
    tcp_thread.start()
    logger.info("TCP receiver thread started")
    
    # Start processing loop thread
    processing_thread = threading.Thread(
        target=processing_loop_thread,
        args=(survivor_manager,),
        name="ProcessingLoop",
        daemon=True
    )
    processing_thread.start()
    logger.info("Processing loop thread started")
    
    # Start WebSocket event broadcaster (asyncio task)
    # This will be handled by FastAPI's lifespan events
    
    logger.info("Backend skeleton ready")
    logger.info("FastAPI server will start on uvicorn.run()")
    logger.info("Press Ctrl+C to shutdown")


@app.on_event("startup")
async def startup_event():
    """Start background tasks on FastAPI startup."""
    logger.info("FastAPI startup: Starting WebSocket event broadcaster")
    asyncio.create_task(websocket_event_broadcaster())


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on FastAPI shutdown."""
    logger.info("FastAPI shutdown: Setting shutdown flag")
    shutdown_flag.set()


if __name__ == "__main__":
    import uvicorn
    
    # Start threads
    main()
    
    # Start FastAPI server (Uvicorn manages its own event loop)
    # Note: Uvicorn spawns its own event loop - no manual threading needed
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )
