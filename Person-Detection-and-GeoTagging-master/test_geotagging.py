#!/usr/bin/env python3
"""
Geotagging Diagnostic and Testing Tool

This script helps you:
1. Test coordinate transformations with known test points
2. Visualize ray casting geometry
3. Validate your camera FOV settings
4. Debug pitch/heading/roll angles
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import yaml

# =====================================================
# CONSTANTS
# =====================================================
R_EARTH = 6378137.0

# =====================================================
# COORDINATE FUNCTIONS (same as maingeo.py)
# =====================================================

def pixel_to_ray(u, v, w, h, hfov, vfov):
    """Convert pixel to normalized ray"""
    x_ndc = (u - w / 2) / (w / 2)
    y_ndc = (v - h / 2) / (h / 2)
    
    ray = np.array([
        math.tan(math.radians(hfov / 2)) * x_ndc,
        math.tan(math.radians(vfov / 2)) * y_ndc,
        1.0
    ])
    return ray / np.linalg.norm(ray)


def rotate_ray(ray, heading_deg, pitch_deg, roll_deg):
    """Rotate ray from camera to world frame (NED)"""
    yaw = math.radians(heading_deg)
    pitch = math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    
    # Camera to body
    R_cam_to_body = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, 1]
    ])
    
    # Roll, Pitch, Yaw
    R_roll = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    
    R_pitch = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    
    R_yaw = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    ray_body = R_cam_to_body @ ray
    ray_world = R_yaw @ R_pitch @ R_roll @ ray_body
    
    return ray_world


def meters_to_latlon(north, east, lat, lon):
    """Convert NED offset to GPS"""
    dlat = north / R_EARTH
    dlon = east / (R_EARTH * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def haversine_distance(lat1, lon1, lat2, lon2):
    """Distance between GPS coordinates"""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    return R_EARTH * c

# =====================================================
# TEST SCENARIOS
# =====================================================

def test_scenario_1_looking_straight_down():
    """
    Test Case 1: Drone looking straight down
    - Altitude: 20m
    - Heading: 0° (North)
    - Pitch: 0° (level flight)
    - Target pixel: Center of image
    
    Expected: Detection directly below drone
    """
    print("\n" + "="*60)
    print("TEST 1: Looking Straight Down (Level Flight)")
    print("="*60)
    
    # Drone state
    drone_lat, drone_lon = 26.4499, 80.3319  # Kanpur
    altitude = 20.0  # meters
    heading = 0.0  # North
    pitch = 0.0    # Level
    roll = 0.0
    
    # Camera
    width, height = 1280, 720
    hfov, vfov = 143.0, 81.0
    
    # Center pixel (looking down through center)
    u, v = width / 2, height / 2
    
    # Calculate
    ray = pixel_to_ray(u, v, width, height, hfov, vfov)
    ray_world = rotate_ray(ray, heading, pitch, roll)
    
    print(f"\nDrone Position: ({drone_lat:.6f}, {drone_lon:.6f})")
    print(f"Altitude: {altitude}m")
    print(f"Orientation: Heading={heading}°, Pitch={pitch}°, Roll={roll}°")
    print(f"\nRay (camera frame): {ray}")
    print(f"Ray (world NED):    {ray_world}")
    
    if ray_world[2] > 0:
        t = altitude / ray_world[2]
        north = ray_world[0] * t
        east = ray_world[1] * t
        dist = math.hypot(north, east)
        
        target_lat, target_lon = meters_to_latlon(north, east, drone_lat, drone_lon)
        
        print(f"\nGround intersection:")
        print(f"  North offset: {north:.2f}m")
        print(f"  East offset:  {east:.2f}m")
        print(f"  Distance:     {dist:.2f}m")
        print(f"  Target GPS:   ({target_lat:.6f}, {target_lon:.6f})")
        print(f"\n✓ EXPECTED: North~0m, East~0m (directly below)")
    else:
        print("\n✗ ERROR: Ray pointing upward!")


def test_scenario_2_pitched_forward():
    """
    Test Case 2: Drone pitched forward while flying
    - Altitude: 20m
    - Heading: 90° (East)
    - Pitch: -10° (nose up 10°, typical forward flight)
    - Target pixel: Center of image
    
    Expected: Detection ahead of drone path
    """
    print("\n" + "="*60)
    print("TEST 2: Forward Flight (Nose Up 10°)")
    print("="*60)
    
    drone_lat, drone_lon = 26.4499, 80.3319
    altitude = 20.0
    heading = 90.0   # Flying East
    pitch = -10.0    # Nose up (typical forward flight)
    roll = 0.0
    
    width, height = 1280, 720
    hfov, vfov = 143.0, 81.0
    
    # Center pixel
    u, v = width / 2, height / 2
    
    ray = pixel_to_ray(u, v, width, height, hfov, vfov)
    ray_world = rotate_ray(ray, heading, pitch, roll)
    
    print(f"\nDrone Position: ({drone_lat:.6f}, {drone_lon:.6f})")
    print(f"Altitude: {altitude}m")
    print(f"Orientation: Heading={heading}°, Pitch={pitch}°, Roll={roll}°")
    print(f"\nRay (camera frame): {ray}")
    print(f"Ray (world NED):    {ray_world}")
    
    if ray_world[2] > 0:
        t = altitude / ray_world[2]
        north = ray_world[0] * t
        east = ray_world[1] * t
        dist = math.hypot(north, east)
        
        target_lat, target_lon = meters_to_latlon(north, east, drone_lat, drone_lon)
        
        print(f"\nGround intersection:")
        print(f"  North offset: {north:.2f}m")
        print(f"  East offset:  {east:.2f}m")
        print(f"  Distance:     {dist:.2f}m")
        print(f"  Target GPS:   ({target_lat:.6f}, {target_lon:.6f})")
        print(f"\n✓ EXPECTED: East offset > 0 (ahead in flight direction)")
    else:
        print("\n✗ ERROR: Ray pointing upward!")


def test_scenario_3_edge_detection():
    """
    Test Case 3: Detection at image edge
    - Tests FOV calculations
    """
    print("\n" + "="*60)
    print("TEST 3: Detection at Image Edge (FOV Validation)")
    print("="*60)
    
    drone_lat, drone_lon = 26.4499, 80.3319
    altitude = 20.0
    heading = 0.0
    pitch = 0.0
    roll = 0.0
    
    width, height = 1280, 720
    hfov, vfov = 143.0, 81.0
    
    # Test points
    test_points = [
        ("Center", width/2, height/2),
        ("Left Edge", 0, height/2),
        ("Right Edge", width, height/2),
        ("Top Edge", width/2, 0),
        ("Bottom Edge", width/2, height),
    ]
    
    print(f"\nDrone at ({drone_lat:.6f}, {drone_lon:.6f}), {altitude}m AGL")
    print(f"Camera FOV: {hfov}° × {vfov}°\n")
    
    for name, u, v in test_points:
        ray = pixel_to_ray(u, v, width, height, hfov, vfov)
        ray_world = rotate_ray(ray, heading, pitch, roll)
        
        if ray_world[2] > 0:
            t = altitude / ray_world[2]
            north = ray_world[0] * t
            east = ray_world[1] * t
            dist = math.hypot(north, east)
            
            print(f"{name:15s}: N={north:6.1f}m, E={east:6.1f}m, Dist={dist:5.1f}m")


def visualize_3d_geometry():
    """
    Create 3D visualization of ray casting geometry
    """
    print("\n" + "="*60)
    print("Generating 3D Visualization...")
    print("="*60)
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Drone position
    drone_pos = np.array([0, 0, 20])  # 20m altitude
    
    # Test different orientations
    test_cases = [
        ("Level (0° pitch)", 0, 0),
        ("Nose up 10°", 0, -10),
        ("Nose down 10°", 0, 10),
        ("Heading 45°", 45, 0),
    ]
    
    width, height = 1280, 720
    hfov, vfov = 143.0, 81.0
    
    colors = ['blue', 'green', 'red', 'purple']
    
    for (name, heading, pitch), color in zip(test_cases, colors):
        # Center ray
        u, v = width/2, height/2
        ray = pixel_to_ray(u, v, width, height, hfov, vfov)
        ray_world = rotate_ray(ray, heading, pitch, 0)
        
        if ray_world[2] > 0:
            t = 20.0 / ray_world[2]
            end_point = drone_pos + ray_world * t
            
            # Draw ray
            ax.plot([drone_pos[0], end_point[0]],
                   [drone_pos[1], end_point[1]],
                   [drone_pos[2], end_point[2]],
                   color=color, label=name, linewidth=2)
            
            # Mark ground intersection
            ax.scatter(end_point[0], end_point[1], 0, 
                      color=color, s=100, marker='o')
    
    # Draw drone
    ax.scatter(0, 0, 20, color='black', s=200, marker='^', label='Drone')
    
    # Ground plane
    xx, yy = np.meshgrid(range(-50, 51, 10), range(-50, 51, 10))
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.2, color='gray')
    
    ax.set_xlabel('North (m)')
    ax.set_ylabel('East (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('Drone Geotagging Ray Geometry')
    ax.legend()
    
    plt.tight_layout()
    plt.savefig('outputs/ray_geometry_3d.png', dpi=150)
    print("✓ Saved: outputs/ray_geometry_3d.png")


def validate_config(config_path="config.yaml"):
    """
    Validate config file settings
    """
    print("\n" + "="*60)
    print("Config File Validation")
    print("="*60)
    
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    
    print(f"\n✓ Camera FOV: {cfg['camera']['hfov_deg']}° × {cfg['camera']['vfov_deg']}°")
    print(f"✓ YOLO model: {cfg['yolo']['model_path']}")
    print(f"✓ Confidence: {cfg['yolo']['confidence_threshold']}")
    print(f"✓ UDP ports: {cfg['network']['udp_port']}, {cfg['network']['udp_port']+1}")
    print(f"✓ Max geotag distance: {cfg['validation']['max_geotag_distance']}m")
    
    # Check for common issues
    if cfg['camera']['hfov_deg'] > 180 or cfg['camera']['vfov_deg'] > 180:
        print("\n⚠ WARNING: FOV > 180° may cause issues")
    
    if cfg['yolo']['confidence_threshold'] < 0.15:
        print("\n⚠ WARNING: Very low confidence threshold may cause false positives")
    
    if cfg['validation']['max_geotag_distance'] < 50:
        print("\n⚠ WARNING: Max geotag distance seems low for typical flight altitudes")


# =====================================================
# MAIN
# =====================================================

def main():
    print("\n" + "="*60)
    print("DRONE GEOTAGGING DIAGNOSTIC TOOL")
    print("="*60)
    
    # Run tests
    test_scenario_1_looking_straight_down()
    test_scenario_2_pitched_forward()
    test_scenario_3_edge_detection()
    
    # Visualize
    visualize_3d_geometry()
    
    # Validate config
    try:
        validate_config()
    except FileNotFoundError:
        print("\n⚠ config.yaml not found - skipping validation")
    
    print("\n" + "="*60)
    print("Diagnostic complete!")
    print("="*60)
    print("\nNext steps:")
    print("1. Review test outputs above")
    print("2. Check 3D visualization: outputs/ray_geometry_3d.png")
    print("3. If results look wrong, adjust camera FOV or rotation matrices")
    print("4. Test with real flight data to validate accuracy")


if __name__ == "__main__":
    main()