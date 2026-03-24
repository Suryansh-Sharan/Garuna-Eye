# Complete Testing & Calibration Guide

## 📋 Table of Contents
1. [Camera Calibration](#1-camera-calibration)
2. [Static Bench Test](#2-static-bench-test)
3. [Indoor Hand Testing](#3-indoor-hand-testing)
4. [Pre-flight Validation](#4-pre-flight-validation)
5. [Troubleshooting](#5-troubleshooting)

---

## 1. Camera Calibration

### Why Calibrate?
The DJI Action 2 has a 143° ultra-wide lens which causes **barrel distortion** (straight lines appear curved). Calibration corrects this for accurate geotagging.

### What You Need
- 📄 Printed checkerboard pattern (9x6 internal corners)
  - Download: https://docs.opencv.org/4.x/pattern.png
  - Print on A4 paper (each square = 25mm)
  - Mount on flat cardboard
- 💻 Laptop with camera connected
- ⏱️ 10-15 minutes

### Step 1.1: Print Checkerboard
```bash
# Download pattern
wget https://raw.githubusercontent.com/opencv/opencv/master/doc/pattern.png

# Print this on A4 paper
# Measure one square to confirm it's 25mm (adjust SQUARE_SIZE in script if different)
```

### Step 1.2: Capture Calibration Images
```bash
python camera_calibration.py --capture --camera 0 --num-images 25
```

**Instructions during capture:**
- Hold checkerboard at different angles (tilted, rotated)
- Different distances (fill 30-80% of frame)
- Move around all corners of the image
- **Wait for green overlay** before pressing SPACE
- Need 25 good images minimum

**Tips:**
- Keep checkerboard flat (use stiff cardboard backing)
- Good lighting (avoid shadows on pattern)
- Hold steady when capturing
- If corners not detected → adjust lighting or distance

### Step 1.3: Run Calibration
```bash
python camera_calibration.py --calibrate --test
```

This will:
- Process all captured images
- Calculate camera matrix & distortion coefficients
- Show reprojection error (should be < 1.0 pixel)
- Display before/after comparison

**Expected Output:**
```
CALIBRATION RESULTS
==================
Reprojection Error: 0.42 pixels  ← Should be < 1.0
Image Size: (1280, 720)

Camera Matrix:
[[682.5    0.0  640.0]
 [  0.0  682.5  360.0]
 [  0.0    0.0    1.0]]

Distortion Coefficients:
[[-0.28  0.15  0.0  0.0  -0.08]]

Calculated FOV:
  Horizontal: 143.2°  ← Should match spec (143°)
  Vertical:   81.1°   ← Should match spec (81°)
```

**Good calibration indicators:**
- ✅ Reprojection error < 1.0 pixel
- ✅ FOV values close to manufacturer specs
- ✅ 15+ images successfully processed
- ✅ Straight lines look straight in undistorted view

### Step 1.4: Update Config
```bash
python camera_calibration.py --update-config
```

This automatically updates `config.yaml` with calibration data.

---

## 2. Static Bench Test

Test coordinate transformations **before** flying. Place drone on table.

### What You Need
- 🔧 Assembled drone (Pixhawk + GPS + Camera)
- 📡 Telemetry connected to laptop (Mission Planner)
- 📍 GPS 3D fix (wait 5-10 minutes)
- 📏 Tape measure
- 🎯 Test targets at known distances

### Step 2.1: Setup Test Environment
```
Your Room Layout:
┌─────────────────────────────┐
│                             │
│    [Person/Object]          │  ← 2-3 meters away
│           ↑                 │
│           │ Known distance  │
│           │                 │
│    [Drone on Table] ←───────┼─── Note GPS position
│                             │
└─────────────────────────────┘
```

1. Place drone on stable table (1-1.5m height)
2. Point camera at test object (person, chair, etc.)
3. Measure exact distance with tape measure
4. Note GPS coordinates from Mission Planner
5. Take photo and compare geotagged position vs. measured

### Step 2.2: Verify GPS Lock
In Mission Planner:
```
Flight Data tab:
  GPS Status: 3D Fix (green)
  Satellites: 8+ (more is better)
  HDOP: < 2.0 (lower is better)
  Altitude: Should match known elevation
```

### Step 2.3: Run Static Test
```bash
# Start Raspberry Pi sender
python3 raspberry_pi_sender.py \
  --laptop-ip 100.115.66.25 \
  --pixhawk-port /dev/ttyACM0 \
  --baudrate 115200 \
  --camera-index 0

# On laptop, run receiver
python maingeo.py --config config.yaml --mode live --show
```

### Step 2.4: Validate Results
1. Point camera at test target
2. Wait for detection
3. Check `outputs/maps/geotag_map_*.html`
4. Measure distance on map vs. actual distance

**Expected Accuracy:**
- Indoor (5m range): ±1-2 meters
- Outdoor (20m range): ±2-3 meters
- High altitude (50m+): ±5-10 meters

**Error sources:**
- GPS error: ±3-5m (civilian GPS)
- Barometer drift: ±1-2m
- Compass calibration: ±5-10°
- Camera FOV uncertainty: ±1-2°

### Step 2.5: Test Different Orientations

Place objects at different positions and test:

| Position | Expected Behavior |
|----------|-------------------|
| Directly ahead (center) | N≈0, E≈measured distance |
| Left edge of frame | N≈0, E=negative |
| Right edge of frame | N≈0, E=positive |

Rotate drone 90° and repeat.

---

## 3. Indoor Hand Testing

Test system **without flying** by moving drone with hands.

### What You Need
- 🤲 Assembled drone (can hold in hands)
- 📍 Your room GPS coordinates (use Google Maps)
- 👥 Person/object to detect
- ⌨️ Keyboard controls

### Step 3.1: Get Your Room Coordinates

**Option A: Google Maps**
1. Go to https://maps.google.com
2. Right-click your room location
3. Click first item (coordinates)
4. Example: `26.4499, 80.3319`

**Option B: GPS Module**
1. Place drone near window
2. Wait for GPS lock
3. Read coordinates from Mission Planner

### Step 3.2: Update Script
Edit `manual_test.py`:
```python
# Line 25-27
BASE_LAT = 26.4499  # ← Your latitude
BASE_LON = 80.3319  # ← Your longitude
BASE_ALT = 1.5      # Height you'll hold drone (meters)
```

### Step 3.3: Run Manual Test
```bash
python manual_test.py
```

### Step 3.4: Testing Procedure

**Phase 1: Static Detection**
1. Hold drone steady pointing at person
2. Press SPACE to capture & detect
3. Check console for geotagged coordinates
4. Note detection distance

**Phase 2: Movement Test**
1. Press W to "move north" 0.5m
2. Press SPACE to detect again
3. Press S to "move south" back to start
4. Repeat with A/D (west/east)

**Phase 3: Rotation Test**
1. Press Q to rotate left 10°
2. Press SPACE to detect
3. Notice how detection position changes
4. Press E to rotate right

**Phase 4: Pitch Test**
1. Press R to pitch up
2. Detection should move farther away
3. Press F to pitch down
4. Detection should move closer

**Expected behavior:**
- Moving north → detections shift south (relative)
- Turning left → detections shift right (in world frame)
- Pitching up → detections appear farther
- Same person should NOT be detected twice if deduplication working

### Step 3.5: Analyze Results
```bash
# Open generated map
firefox outputs/maps/manual_test_final_*.html

# Check:
# - Blue line = drone path
# - Red circles = detections
# - Does path make sense?
# - Are detections in expected locations?
```

---

## 4. Pre-Flight Validation

Final checks before actual flight test.

### Checklist

**Hardware:**
- [ ] GPS has 3D fix (8+ satellites)
- [ ] Compass calibrated (do figure-8 pattern)
- [ ] Barometer shows correct altitude
- [ ] Camera connected and working
- [ ] Raspberry Pi online (check Tailscale)
- [ ] Battery charged (>50%)

**Software:**
- [ ] Laptop receiving telemetry (check terminal)
- [ ] Laptop receiving images (check terminal)
- [ ] YOLO model loading successfully
- [ ] No errors in logs
- [ ] Map output directory exists

**Telemetry Check:**
```bash
# On laptop terminal, should see:
INFO - Telemetry receiver listening on 0.0.0.0:6000
INFO - Image receiver listening on 0.0.0.0:6001

# Then when RPI sends data:
INFO - Frame #1: X detection(s)
INFO - New detection at (lat, lon) - Xm away
```

**Network Check:**
```bash
# From laptop, ping Raspberry Pi
ping 100.x.x.x  # Your RPI Tailscale IP

# From RPI, ping laptop
ping 100.115.66.25  # Your laptop IP

# Both should respond < 50ms
```

**Safety Check:**
- [ ] Manual control works (test in stabilize mode)
- [ ] Failsafe configured (RTL on signal loss)
- [ ] Home position set correctly
- [ ] Flight area clear of obstacles
- [ ] Emergency stop plan in place

---

## 5. Troubleshooting

### Problem: No Detections

**Possible causes:**
1. YOLO confidence too high
   ```yaml
   # In config.yaml, lower threshold
   confidence_threshold: 0.15  # Was 0.25
   ```

2. Wrong class names
   ```bash
   # Check your model's classes
   python -c "from ultralytics import YOLO; m=YOLO('your_model.pt'); print(m.names)"
   ```

3. Image quality too low
   ```bash
   # Increase JPEG quality in RPI sender
   --image-quality 80  # Was 40
   ```

### Problem: Detections in Wrong Location

**Check 1: Heading Direction**
```bash
# In Mission Planner, check:
# - Heading matches drone orientation
# - 0° = North, 90° = East, 180° = South, 270° = West
# - If wrong, recalibrate compass
```

**Check 2: Pitch Angle**
```bash
# Check if pitch sign is correct:
# - Negative pitch = nose up
# - Positive pitch = nose down
# - Level flight ≈ -5° to -10° (nose slightly up)
```

**Check 3: Altitude**
```bash
# Verify altitude in Mission Planner:
# - Should show AGL (above ground level)
# - Not AMSL (above mean sea level)
# - Use rangefinder for <10m altitude
```

**Check 4: FOV Values**
```yaml
# If objects appear too close/far, adjust FOV
camera:
  hfov_deg: 140.0  # Try ±5° adjustments
  vfov_deg: 78.0
```

### Problem: Same Person Detected Multiple Times

**Solution: Increase deduplication**
```yaml
deduplication:
  min_distance_meters: 5.0   # Increase from 3.0
  time_window_seconds: 60.0  # Increase from 30.0
```

### Problem: GPS Jumping Around

**Causes:**
- Poor satellite visibility → fly in open area
- Electromagnetic interference → keep GPS away from motors/ESCs
- Bad GPS module → check connections

**Solutions:**
1. Wait longer for GPS lock (10+ satellites)
2. Enable GPS filtering in Pixhawk
3. Use external GPS with better antenna

### Problem: High Latency

**Check network:**
```bash
# Measure ping
ping -c 10 100.x.x.x

# Should be < 50ms
# If > 100ms, check WiFi/Tailscale
```

**Reduce data rate:**
```bash
# Lower FPS
--fps 0.5  # Was 1.0

# Lower resolution (edit camera_capture.py)
width=640, height=480  # Was 1280x720

# Lower JPEG quality
--image-quality 30  # Was 40
```

### Problem: "Ray pointing upward"

This means drone pitch/roll is too extreme.

**Solutions:**
1. Fly more level (reduce pitch)
2. Adjust pitch limits in config:
   ```yaml
   validation:
     min_pitch_deg: -60.0  # Allow more nose-up
     max_pitch_deg: 60.0   # Allow more nose-down
   ```

---

## Testing Timeline

**Day 1: Calibration (30 min)**
- Camera calibration
- Config update

**Day 2: Static Test (1 hour)**
- Bench test with measured distances
- Validate accuracy

**Day 3: Indoor Test (30 min)**
- Manual hand movement test
- Verify coordinate transformations

**Day 4: Ground Test (1 hour)**
- Place drone on ground
- Test with walking people
- Verify before flight

**Day 5: First Flight (2 hours)**
- Low altitude test (5m)
- Simple waypoint pattern
- Review results

---

## Success Criteria

✅ **Calibration:**
- Reprojection error < 1.0 pixel
- FOV matches specifications

✅ **Static Test:**
- Detection distance within ±2m of actual
- Consistent results over 10+ tests

✅ **Manual Test:**
- Coordinate transformations logical
- Movement directions correct
- No crashes/errors

✅ **Flight Test:**
- Safe takeoff/landing
- Continuous telemetry link
- Detections recorded
- Map generated successfully

---

## Quick Reference

```bash
# 1. Calibrate camera
python camera_calibration.py --capture
python camera_calibration.py --calibrate --test
python camera_calibration.py --update-config

# 2. Run diagnostic
python test_geotagging.py

# 3. Indoor test
python manual_test.py

# 4. Live flight
# On RPI:
python3 raspberry_pi_sender.py --laptop-ip 100.115.66.25 ...

# On Laptop:
python maingeo.py --config config.yaml --mode live --show
```

---

## Getting Help

If stuck:
1. Check logs: `tail -f drone_geotag.log`
2. Run diagnostic: `python test_geotagging.py`
3. Verify each component separately
4. Review error messages carefully

Good luck! 🚁