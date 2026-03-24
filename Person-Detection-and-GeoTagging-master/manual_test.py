#!/usr/bin/env python3
"""
Manual Indoor Testing Script

Test the geotagging system by moving drone with your hands indoors.
This simulates GPS fixes and validates detection + coordinate transformation.
"""

import cv2
import json
import time
import numpy as np
import folium
from dataclasses import dataclass
from typing import List, Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import from maingeo
import sys
sys.path.append('.')

from maingeo2 import (
    PeopleGeotagger, Config, DroneState,
    pixel_to_ray, rotate_ray, meters_to_latlon
)

# =====================================================
# SIMULATED GPS POSITION (Your room location)
# =====================================================

# Replace with your actual location (use Google Maps to get coordinates)
BASE_LAT = 26.4499  # Kanpur, India
BASE_LON = 80.3319
BASE_ALT = 1.5      # Simulated altitude (1.5m - holding drone at chest height)

# =====================================================
# MANUAL CONTROL INTERFACE
# =====================================================

class ManualDroneController:
    """
    Simulate drone movement using keyboard/mouse
    """
    
    def __init__(self, base_lat, base_lon, base_alt):
        self.lat = base_lat
        self.lon = base_lon
        self.alt = base_alt
        self.heading = 0.0  # North
        self.pitch = 0.0    # Level
        self.roll = 0.0
        
        # Movement increments
        self.move_step = 0.5  # meters
        self.heading_step = 10.0  # degrees
        self.pitch_step = 5.0  # degrees
        
        # Convert meters to lat/lon deltas
        self.lat_per_meter = 1.0 / 111320.0
        self.lon_per_meter = 1.0 / (111320.0 * np.cos(np.radians(base_lat)))
    
    def get_state(self) -> DroneState:
        """Get current drone state"""
        return DroneState(
            lat=self.lat,
            lon=self.lon,
            alt_agl=self.alt,
            heading_deg=self.heading,
            pitch_deg=self.pitch,
            roll_deg=self.roll,
            timestamp=time.time()
        )
    
    def move_north(self):
        self.lat += self.move_step * self.lat_per_meter
        logger.info(f"Moved NORTH → {self.lat:.7f}, {self.lon:.7f}")
    
    def move_south(self):
        self.lat -= self.move_step * self.lat_per_meter
        logger.info(f"Moved SOUTH → {self.lat:.7f}, {self.lon:.7f}")
    
    def move_east(self):
        self.lon += self.move_step * self.lon_per_meter
        logger.info(f"Moved EAST → {self.lat:.7f}, {self.lon:.7f}")
    
    def move_west(self):
        self.lon -= self.move_step * self.lon_per_meter
        logger.info(f"Moved WEST → {self.lat:.7f}, {self.lon:.7f}")
    
    def turn_left(self):
        self.heading = (self.heading - self.heading_step) % 360
        logger.info(f"Turned LEFT → {self.heading:.0f}°")
    
    def turn_right(self):
        self.heading = (self.heading + self.heading_step) % 360
        logger.info(f"Turned RIGHT → {self.heading:.0f}°")
    
    def pitch_up(self):
        self.pitch = max(-30, self.pitch - self.pitch_step)
        logger.info(f"Pitched UP → {self.pitch:.0f}°")
    
    def pitch_down(self):
        self.pitch = min(30, self.pitch + self.pitch_step)
        logger.info(f"Pitched DOWN → {self.pitch:.0f}°")
    
    def increase_alt(self):
        self.alt += 0.5
        logger.info(f"Altitude increased → {self.alt:.1f}m")
    
    def decrease_alt(self):
        self.alt = max(0.5, self.alt - 0.5)
        logger.info(f"Altitude decreased → {self.alt:.1f}m")
    
    def reset(self):
        self.heading = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        logger.info("Orientation RESET → N:0° P:0° R:0°")

# =====================================================
# VISUALIZATION
# =====================================================

def create_test_map(detections: List[dict], drone_states: List[DroneState], 
                    base_lat, base_lon):
    """
    Create interactive map showing drone path and detections
    """
    
    if not drone_states:
        return None
    
    # Center map on average position
    center_lat = np.mean([d.lat for d in drone_states])
    center_lon = np.mean([d.lon for d in drone_states])
    
    m = folium.Map(
        location=[center_lat, center_lon],
        zoom_start=20,  # Very close zoom for indoor testing
        tiles='OpenStreetMap'
    )
    
    # Draw drone path
    path_coords = [(d.lat, d.lon) for d in drone_states]
    folium.PolyLine(
        path_coords,
        color='blue',
        weight=2,
        opacity=0.7,
        popup='Drone Path'
    ).add_to(m)
    
    # Mark start and end
    if len(drone_states) > 0:
        start = drone_states[0]
        end = drone_states[-1]
        
        folium.Marker(
            [start.lat, start.lon],
            popup='START',
            icon=folium.Icon(color='green', icon='play')
        ).add_to(m)
        
        folium.Marker(
            [end.lat, end.lon],
            popup=f'END<br>Alt:{end.alt_agl:.1f}m<br>Hdg:{end.heading_deg:.0f}°',
            icon=folium.Icon(color='red', icon='stop')
        ).add_to(m)
    
    # Mark detections
    for i, det in enumerate(detections, 1):
        folium.CircleMarker(
            [det['lat'], det['lon']],
            radius=8,
            color='red',
            fill=True,
            fillColor='red',
            fillOpacity=0.8,
            popup=f"Person #{i}<br>Dist: {det['distance']:.1f}m<br>Conf: {det['confidence']:.2f}"
        ).add_to(m)
    
    # Add reference point (your room location)
    folium.Marker(
        [base_lat, base_lon],
        popup='Room Origin',
        icon=folium.Icon(color='gray', icon='home')
    ).add_to(m)
    
    return m

# =====================================================
# MAIN TEST INTERFACE
# =====================================================

def main():
    print("\n" + "="*70)
    print(" "*15 + "MANUAL INDOOR TESTING MODE")
    print("="*70)
    print("\n📍 Setup Instructions:")
    print("  1. Connect camera to laptop")
    print("  2. Place drone assembly at known location")
    print("  3. Point camera at people/objects to detect")
    print("  4. Use keyboard to simulate drone movement")
    print("\n⌨️  Controls:")
    print("  W/S/A/D     - Move North/South/West/East")
    print("  Q/E         - Turn Left/Right (heading)")
    print("  R/F         - Pitch Up/Down")
    print("  T/G         - Increase/Decrease Altitude")
    print("  SPACE       - Capture & Process Frame")
    print("  0 (zero)    - Reset orientation")
    print("  M           - Save map")
    print("  ESC         - Quit")
    print("="*70)
    
    # Load config
    try:
        config = Config("config.yaml")
        print("✓ Loaded config.yaml")
    except Exception as e:
        print(f"✗ Error loading config: {e}")
        return
    
    # Initialize geotagger
    geotagger = PeopleGeotagger(config)
    print("✓ YOLO model loaded")
    
    # Initialize camera
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("✗ Cannot open camera")
        return
    
    print("✓ Camera opened")
    
    # Initialize controller
    controller = ManualDroneController(BASE_LAT, BASE_LON, BASE_ALT)
    print(f"✓ Starting position: ({BASE_LAT:.6f}, {BASE_LON:.6f})")
    
    # Storage
    all_detections = []
    drone_states = []
    frame_count = 0
    
    print("\n🚀 Ready! Press SPACE to start detecting...")
    
    while True:
        # Capture frame
        ret, frame = camera.read()
        if not ret:
            print("✗ Failed to capture frame")
            break
        
        # Get current state
        drone_state = controller.get_state()
        
        # Create display
        display = frame.copy()
        h, w = display.shape[:2]
        
        # Overlay telemetry
        info_lines = [
            f"Position: ({drone_state.lat:.7f}, {drone_state.lon:.7f})",
            f"Altitude: {drone_state.alt_agl:.1f}m",
            f"Heading: {drone_state.heading_deg:.0f}° | Pitch: {drone_state.pitch_deg:.0f}°",
            f"Detections: {len(all_detections)} | Frames: {frame_count}",
            "",
            "Press SPACE to detect | ESC to quit | M for map"
        ]
        
        y_offset = 30
        for line in info_lines:
            # Black outline
            cv2.putText(display, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            # White text
            cv2.putText(display, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            y_offset += 30
        
        # Draw crosshair (image center)
        center_x, center_y = w // 2, h // 2
        cv2.line(display, (center_x - 20, center_y), (center_x + 20, center_y), 
                (0, 255, 0), 2)
        cv2.line(display, (center_x, center_y - 20), (center_x, center_y + 20), 
                (0, 255, 0), 2)
        
        cv2.imshow("Manual Testing", display)
        
        # Handle keyboard
        key = cv2.waitKey(1) & 0xFF
        
        if key == 27:  # ESC
            break
        
        elif key == ord(' '):  # SPACE - Detect
            frame_count += 1
            logger.info(f"\n{'='*50}")
            logger.info(f"FRAME #{frame_count} - Processing...")
            logger.info(f"{'='*50}")
            
            # Run detection
            detections = geotagger.detect(frame)
            logger.info(f"Detected {len(detections)} objects")
            
            if detections:
                # Geotag
                geotagged = geotagger.geotag(detections, frame, drone_state)
                logger.info(f"Geotagged {len(geotagged)} valid detections")
                
                # Store
                all_detections.extend(geotagged)
                drone_states.append(drone_state)
                
                # Draw on frame
                for det in geotagged:
                    x1, y1, x2, y2 = map(int, det['bbox'])
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    label = f"{det['distance']:.1f}m"
                    cv2.putText(display, label, (x1, y1-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    logger.info(f"  → ({det['lat']:.7f}, {det['lon']:.7f}) - {det['distance']:.1f}m away")
                
                cv2.imshow("Manual Testing", display)
                cv2.waitKey(1000)  # Show for 1 second
            else:
                logger.info("No detections in this frame")
        
        # Movement controls
        elif key == ord('w'):
            controller.move_north()
        elif key == ord('s'):
            controller.move_south()
        elif key == ord('a'):
            controller.move_west()
        elif key == ord('d'):
            controller.move_east()
        
        # Rotation controls
        elif key == ord('q'):
            controller.turn_left()
        elif key == ord('e'):
            controller.turn_right()
        
        # Pitch controls
        elif key == ord('r'):
            controller.pitch_up()
        elif key == ord('f'):
            controller.pitch_down()
        
        # Altitude controls
        elif key == ord('t'):
            controller.increase_alt()
        elif key == ord('g'):
            controller.decrease_alt()
        
        # Reset
        elif key == ord('0'):
            controller.reset()
        
        # Save map
        elif key == ord('m'):
            if all_detections:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                map_file = f"outputs/maps/manual_test_{timestamp}.html"
                
                test_map = create_test_map(all_detections, drone_states, 
                                          BASE_LAT, BASE_LON)
                if test_map:
                    test_map.save(map_file)
                    logger.info(f"\n✓ Map saved: {map_file}")
                    logger.info(f"  Open in browser to view results")
            else:
                logger.info("\n⚠ No detections yet - capture some frames first!")
    
    # Cleanup
    camera.release()
    cv2.destroyAllWindows()
    
    # Final summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Total frames processed: {frame_count}")
    print(f"Total detections: {len(all_detections)}")
    print(f"Drone positions: {len(drone_states)}")
    
    if all_detections:
        print("\nSaving final map...")
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        map_file = f"outputs/maps/manual_test_final_{timestamp}.html"
        
        test_map = create_test_map(all_detections, drone_states, 
                                  BASE_LAT, BASE_LON)
        if test_map:
            test_map.save(map_file)
            print(f"✓ Final map saved: {map_file}")
            
            # Print detection list
            print("\nDetections:")
            for i, det in enumerate(all_detections, 1):
                print(f"  {i}. ({det['lat']:.7f}, {det['lon']:.7f}) - "
                      f"{det['distance']:.1f}m, conf={det['confidence']:.2f}")
    
    print("\n" + "="*70)
    print("Testing complete!")
    print("="*70)


if __name__ == "__main__":
    # Update BASE_LAT and BASE_LON at top of file with your room's location
    print("\n⚠️  IMPORTANT: Update BASE_LAT and BASE_LON in the script")
    print("   with your actual room location (use Google Maps)")
    input("\nPress Enter to continue...")
    
    main()