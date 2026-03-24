import math
import cv2
import numpy as np
from ultralytics import YOLO
import folium

# ===============================================
# CONFIGURATION (edit these manually for testing)
# ===============================================

# DJI Action 2 camera FOV (approx)
HFOV_DEG = 143.0   # horizontal FOV
VFOV_DEG = 81.0    # vertical FOV

# Drone telemetry (you'll replace later with Pixhawk data)
DRONE_LAT = 28.6139       # degrees
DRONE_LON = 77.2090       # degrees
DRONE_ALT_AGL = 20.0      # meters above ground
DRONE_HEADING_DEG = 45.0  # degrees (0°=North, 90°=East)

# YOLO model weights path
MODEL_PATH = "/home/suryansh/All-Coding-FIles/YOLO-Trained/runs/train/yolo11_visdrone_final2/weights/best.pt"

# Input test image
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

def compute_ground_footprint(alt_m, hfov_deg, vfov_deg):
    """Ground width & height (in meters) for given altitude + FOV."""
    hfov_rad = math.radians(hfov_deg)
    vfov_rad = math.radians(vfov_deg)
    width = 2 * alt_m * math.tan(hfov_rad / 2.0)
    height = 2 * alt_m * math.tan(vfov_rad / 2.0)
    return width, height

def bbox_bottom_center(bbox):
    """bbox = (x_min, y_min, x_max, y_max) → bottom-center pixel."""
    x_min, y_min, x_max, y_max = bbox
    u = (x_min + x_max) / 2.0
    v = y_max
    return u, v

def pixel_to_ground_offset(u, v, img_w, img_h, ground_w, ground_h):
    """Convert pixel coordinate to (right, forward) offset in meters."""
    x_norm = (u - img_w / 2.0) / img_w
    y_norm = (img_h / 2.0 - v) / img_h
    right_m = x_norm * ground_w
    forward_m = y_norm * ground_h
    return right_m, forward_m

def body_to_global(right_m, forward_m, heading_deg):
    """Rotate body-frame offsets (forward/right) → global North/East."""
    psi = math.radians(heading_deg)
    north = math.cos(psi) * forward_m - math.sin(psi) * right_m
    east = math.sin(psi) * forward_m + math.cos(psi) * right_m
    return north, east

# ===============================================
# FOLIUM MAP VISUALIZATION
# ===============================================

def plot_geotags_on_map(detections, center_lat, center_lon):
    """Create a folium map and save as HTML file."""
    m = folium.Map(location=[center_lat, center_lon], zoom_start=19)

    # Drone marker
    folium.Marker(
        [center_lat, center_lon],
        popup="Drone Position",
        icon=folium.Icon(color='blue', icon='plane', prefix='fa')
    ).add_to(m)

    # Person markers
    for i, d in enumerate(detections, start=1):
        label = (f"{i}. {d.get('class', 'person').capitalize()}<br>"
                 f"Conf: {d['confidence']:.2f}<br>"
                 f"Lat: {d['lat']:.7f}<br>"
                 f"Lon: {d['lon']:.7f}")
        folium.Marker(
            [d['lat'], d['lon']],
            popup=label,
            icon=folium.Icon(color='red', icon='user', prefix='fa')
        ).add_to(m)

        # Optional: draw line from drone to person
        folium.PolyLine([(center_lat, center_lon), (d['lat'], d['lon'])],
                        color="blue", weight=1).add_to(m)

    m.save("geotag_map.html")
    print("\n🗺️ Map saved as geotag_map.html — open it in your browser.")

# ===============================================
# MAIN FUNCTION
# ===============================================

def geotag_people(image_path, model_path):
    # Load image
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Image not found: {image_path}")
    img_h, img_w = img.shape[:2]

    # Load YOLOv11 model
    model = YOLO(model_path)

    # Run inference
    results = model.predict(source=img, conf=0.25, imgsz=640, verbose=False)[0]

    # Ground footprint
    ground_w, ground_h = compute_ground_footprint(DRONE_ALT_AGL, HFOV_DEG, VFOV_DEG)
    print(f"\nGround footprint: {ground_w:.1f} m × {ground_h:.1f} m at {DRONE_ALT_AGL:.1f} m altitude")

    detections = []
    for box in results.boxes:
        cls_id = int(box.cls[0])
        cls_name = results.names[cls_id].lower()
        if cls_name not in HUMAN_CLASSES:
            continue

        x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
        u, v = bbox_bottom_center((x_min, y_min, x_max, y_max))
        right_m, forward_m = pixel_to_ground_offset(u, v, img_w, img_h, ground_w, ground_h)
        north_m, east_m = body_to_global(right_m, forward_m, DRONE_HEADING_DEG)
        lat, lon = meters_to_latlon_offset(north_m, east_m, DRONE_LAT, DRONE_LON)

        detections.append({
            "class": cls_name,
            "bbox": (x_min, y_min, x_max, y_max),
            "pixel_bottom_center": (u, v),
            "offset_body_m": {"right": right_m, "forward": forward_m},
            "offset_global_m": {"north": north_m, "east": east_m},
            "lat": lat,
            "lon": lon,
            "confidence": float(box.conf[0])
        })

        # Draw bbox
        cv2.rectangle(img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0,255,0), 2)
        cv2.putText(img, f"{cls_name} {box.conf[0]:.2f}", (int(x_min), int(y_min)-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

    # Save annotated image
    cv2.imwrite("output_geotagged.jpg", img)
    print("\n✅ Saved annotated image: output_geotagged.jpg")

    # Print detections
    if not detections:
        print("No human detections found.")
    else:
        for i, d in enumerate(detections, start=1):
            print(f"\nPerson #{i} ({d['class']}):")
            print(f"  Confidence: {d['confidence']:.2f}")
            print(f"  Pixel: {d['pixel_bottom_center']}")
            print(f"  Lat/Lon: {d['lat']:.7f}, {d['lon']:.7f}")

    # Plot on map
    if detections:
        plot_geotags_on_map(detections, DRONE_LAT, DRONE_LON)

    return detections

# ===============================================
# RUN
# ===============================================

if __name__ == "__main__":
    geotag_people(IMAGE_PATH, MODEL_PATH)
