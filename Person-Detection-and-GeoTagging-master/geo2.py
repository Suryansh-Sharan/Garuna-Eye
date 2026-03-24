import math
import cv2
import numpy as np
from ultralytics import YOLO
import folium
from folium.plugins import MarkerCluster

# ===============================================
# CONFIGURATION (edit these manually for testing)
# ===============================================

# DJI Action 2 camera FOV (approx)
HFOV_DEG = 143.0   # horizontal FOV
VFOV_DEG = 81.0    # vertical FOV

# Drone telemetry (replace later with Pixhawk)
DRONE_LAT = 28.6139       # degrees
DRONE_LON = 77.2090       # degrees
DRONE_ALT_AGL = 20.0      # meters above ground
DRONE_HEADING_DEG = 125.0  # yaw (0°=North, 90°=East)
CAMERA_PITCH_DEG = 135.0   # tilt from horizontal (90° = straight down, 0° = forward)

# YOLO model and image paths
MODEL_PATH = "/home/suryansh/All-Coding-FIles/YOLO-Trained/runs/train/yolo11_visdrone_final2/weights/best.pt"
IMAGE_PATH = "/home/suryansh/All-Coding-FIles/YOLO-Trained/Visdrone/Output/images/train/0000068_00001_d_0000001.jpg"

# Human classes from VisDrone dataset
HUMAN_CLASSES = ["person", "people", "pedestrian"]

# ===============================================
# GEO + MATH HELPERS
# ===============================================

R_EARTH = 6378137.0  # radius of Earth (meters)

def meters_to_latlon_offset(north_m, east_m, lat0_deg, lon0_deg):
    """Convert local offsets (north, east in meters) to latitude/longitude."""
    lat0_rad = math.radians(lat0_deg)
    dlat = north_m / R_EARTH
    dlon = east_m / (R_EARTH * math.cos(lat0_rad))
    lat = lat0_deg + math.degrees(dlat)
    lon = lon0_deg + math.degrees(dlon)
    return lat, lon


def pixel_to_ray(u, v, img_w, img_h, hfov_deg, vfov_deg):
    """
    Convert pixel coordinates (u,v) → normalized ray in camera frame.
    Camera frame convention: +Z = forward (optical axis), +X = right, +Y = down.
    """
    hfov_rad = math.radians(hfov_deg)
    vfov_rad = math.radians(vfov_deg)

    # Normalized pixel coordinates in range [-1, 1]
    x_norm = (u - img_w / 2) / (img_w / 2)
    y_norm = (v - img_h / 2) / (img_h / 2)

    # Corresponding angles
    x_angle = x_norm * (hfov_rad / 2)
    y_angle = y_norm * (vfov_rad / 2)

    # Direction vector in camera frame
    x = math.tan(x_angle)
    y = math.tan(y_angle)
    z = 1.0
    ray = np.array([x, y, z])
    ray = ray / np.linalg.norm(ray)
    return ray


def rotate_camera_to_global(ray_cam, heading_deg, pitch_deg):
    """
    Rotate camera ray (camera frame) to world frame (North-East-Down approx).
    - heading_deg: yaw (rotation about Z)
    - pitch_deg: camera tilt down from horizontal (90° = nadir)
    """
    # Convert to radians
    yaw = math.radians(heading_deg)
    pitch = math.radians(pitch_deg)

    # Camera rotation matrix: pitch (about X), then yaw (about Z)
    R_pitch = np.array([
        [1, 0, 0],
        [0, math.cos(pitch), -math.sin(pitch)],
        [0, math.sin(pitch), math.cos(pitch)]
    ])
    R_yaw = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw),  math.cos(yaw), 0],
        [0, 0, 1]
    ])

    R = R_yaw @ R_pitch
    ray_global = R @ ray_cam
    return ray_global


def intersect_ray_with_ground(ray_global, altitude):
    """
    Find where a ray (from camera at height `altitude`) hits the ground (z=0).
    Returns (north, east) offset in meters.
    """
    # camera origin at (0,0,altitude)
    # ray direction: ray_global (Nx, Ny, Nz)
    if ray_global[2] >= 0:
        return None  # Ray points upward, no intersection

    t = -altitude / ray_global[2]
    north = t * ray_global[1]  # Y forward in our rotation convention
    east = t * ray_global[0]   # X right
    return north, east


# ===============================================
# FOLIUM MAP VISUALIZATION
# ===============================================

def plot_geotags_on_map(detections, center_lat, center_lon):
    m = folium.Map(location=[center_lat, center_lon], zoom_start=19)

    # 1. Keep the Drone as a big marker so it stands out
    folium.Marker(
        [center_lat, center_lon],
        popup="Drone Position",
        icon=folium.Icon(color='blue', icon='plane', prefix='fa')
    ).add_to(m)

    for i, d in enumerate(detections, start=1):
        label = (f"{i}. {d.get('class', 'person').capitalize()}<br>"
                 f"Conf: {d['confidence']:.2f}<br>"
                 f"Lat: {d['lat']:.7f}<br>"
                 f"Lon: {d['lon']:.7f}")
        
        # 2. Use CircleMarker for detections
        folium.CircleMarker(
            location=[d['lat'], d['lon']],
            radius=4,              # Size in pixels (Adjust this to make it smaller/larger)
            color='red',           # Border color
            weight=1,              # Border thickness
            fill=True,
            fill_color='red',      # Inner color
            fill_opacity=0.7,
            popup=label
        ).add_to(m)
        
        # Add the line
        folium.PolyLine([(center_lat, center_lon), (d['lat'], d['lon'])],
                        color="blue", weight=1).add_to(m)

    m.save("geotag_map.html")
    print("\n🗺️ Map saved as geotag_map.html")


# ===============================================
# MAIN FUNCTION
# ===============================================

def geotag_people(image_path, model_path):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    img_h, img_w = img.shape[:2]

    model = YOLO(model_path)
    results = model.predict(source=img, conf=0.25, imgsz=640, verbose=False)[0]

    detections = []
    for box in results.boxes:
        cls_id = int(box.cls[0])
        cls_name = results.names[cls_id].lower()
        if cls_name not in HUMAN_CLASSES:
            continue

        # Bottom-center of bbox
        x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
        u, v = (x_min + x_max) / 2, y_max

        # Step 1: pixel → ray in camera frame
        ray_cam = pixel_to_ray(u, v, img_w, img_h, HFOV_DEG, VFOV_DEG)

        # Step 2: rotate to global using yaw + pitch
        ray_global = rotate_camera_to_global(ray_cam, DRONE_HEADING_DEG, CAMERA_PITCH_DEG)

        # Step 3: intersect with ground
        intersect = intersect_ray_with_ground(ray_global, DRONE_ALT_AGL)
        if intersect is None:
            continue
        north_m, east_m = intersect

        # Step 4: convert to lat/lon
        lat, lon = meters_to_latlon_offset(north_m, east_m, DRONE_LAT, DRONE_LON)

        detections.append({
            "class": cls_name,
            "bbox": (x_min, y_min, x_max, y_max),
            "pixel_bottom_center": (u, v),
            "lat": lat,
            "lon": lon,
            "north_m": north_m,
            "east_m": east_m,
            "confidence": float(box.conf[0])
        })

        cv2.rectangle(img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0,255,0), 2)
        cv2.putText(img, f"{cls_name} {box.conf[0]:.2f}", (int(x_min), int(y_min)-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    cv2.imwrite("output_geotagged.jpg", img)
    print("\n✅ Saved annotated image: output_geotagged.jpg")

    if not detections:
        print("No human detections found.")
    else:
        for i, d in enumerate(detections, start=1):
            print(f"\nPerson #{i} ({d['class']}):")
            print(f"  Confidence: {d['confidence']:.2f}")
            print(f"  Pixel: {d['pixel_bottom_center']}")
            print(f"  Ground offsets (N,E): {d['north_m']:.2f}, {d['east_m']:.2f}")
            print(f"  Lat/Lon: {d['lat']:.7f}, {d['lon']:.7f}")

    if detections:
        plot_geotags_on_map(detections, DRONE_LAT, DRONE_LON)

    return detections


# ===============================================
# RUN
# ===============================================

if __name__ == "__main__":
    geotag_people(IMAGE_PATH, MODEL_PATH)
