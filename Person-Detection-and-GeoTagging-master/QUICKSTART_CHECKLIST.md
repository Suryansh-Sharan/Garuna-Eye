# 🚁 Quick Start Checklist

## Your Current Setup
- ✅ Raspberry Pi with Pixhawk (via /dev/ttyACM0)
- ✅ DJI Action 2 camera
- ✅ Mission Planner connected
- ✅ Tailscale VPN (100.115.66.25)
- ✅ YOLO model trained on VisDrone

---

## Step-by-Step Testing (Room Test)

### 1️⃣ System Check (5 min)
```bash
python system_check.py
```
**Expected:** All green checkmarks ✓

---

### 2️⃣ Camera Calibration (15 min)

**Print checkerboard:**
- Download: https://docs.opencv.org/4.x/pattern.png
- Print on A4, mount on cardboard

**Capture & calibrate:**
```bash
# Step 1: Capture 25 images
python camera_calibration.py --capture --num-images 25

# Step 2: Process calibration
python camera_calibration.py --calibrate

# Step 3: Update config
python camera_calibration.py --update-config
```

**Success criteria:**
- Reprojection error < 1.0 pixel
- FOV ≈ 143° × 81°

---

### 3️⃣ Indoor Hand Test (30 min)

**Setup:**
1. Get your room GPS coordinates (Google Maps → right-click → copy coords)
2. Edit `manual_test.py` lines 25-27:
   ```python
   BASE_LAT = 26.4499  # YOUR LATITUDE
   BASE_LON = 80.3319  # YOUR LONGITUDE
   BASE_ALT = 1.5      # Height holding drone (meters)
   ```

**Run test:**
```bash
python manual_test.py
```

**Test procedure:**
1. Hold drone pointing at person
2. Press **SPACE** to detect
3. Press **W/A/S/D** to simulate movement
4. Press **Q/E** to rotate
5. Press **M** to save map
6. Open `outputs/maps/manual_test_*.html`

**Expected results:**
- Person detected with red circle on map
- Detection position makes sense relative to "drone"
- Moving W (north) shifts detection south on map
- Rotating left shifts detection right on map

---

### 4️⃣ Validate Coordinate Math (10 min)

```bash
python test_geotagging.py
```

**Check outputs:**
- Test 1: Center detection → N≈0m, E≈0m ✓
- Test 2: Forward flight → East offset > 0 ✓
- Visualization saved to `outputs/ray_geometry_3d.png`

---

### 5️⃣ Live Test with Hardware (Setup Check)

**On Raspberry Pi:**
```bash
# Check GPS lock in Mission Planner first!
# Need 8+ satellites, 3D fix

python3 raspberry_pi_sender.py \
  --laptop-ip 100.115.66.25 \
  --telemetry-port 6000 \
  --image-port 6001 \
  --pixhawk-port /dev/ttyACM0 \
  --baudrate 115200 \
  --camera-index 0 \
  --fps 1.0 \
  --image-quality 40
```

**On Laptop:**
```bash
python maingeo.py --config config.yaml --mode live --show
```

**What you should see:**
```
INFO - Telemetry receiver listening on 0.0.0.0:6000
INFO - Image receiver listening on 0.0.0.0:6001
INFO - YOLO loaded: runs/train/yolo11_visdrone_final2/weights/best.pt
INFO - LIVE MODE STARTED | q=quit | s=save
```

---

### 6️⃣ Room Test with Real Hardware (15 min)

**Setup:**
1. Place drone on table near window (for GPS)
2. Wait for GPS 3D fix in Mission Planner
3. Have person stand 2-3 meters in front of camera
4. Measure distance with tape measure

**Test:**
1. Start sender (Raspberry Pi)
2. Start receiver (Laptop)
3. Point camera at person
4. Wait for detection
5. Check `outputs/maps/geotag_map_*.html`
6. Compare measured vs. detected distance

**Move drone by hand:**
1. Move left → detection should shift right on map
2. Move forward → detection distance should decrease
3. Rotate drone → detection position changes accordingly

**Success criteria:**
- Detection within ±2m of measured distance
- Movement directions consistent
- No "ray pointing upward" errors
- Map shows logical positions

---

## Common Issues & Quick Fixes

### ❌ No detections
```yaml
# Lower confidence in config.yaml
yolo:
  confidence_threshold: 0.15  # Was 0.25
```

### ❌ Wrong GPS
- Wait longer (GPS needs 10+ min for accurate fix)
- Move drone near window
- Check satellites in Mission Planner (need 8+)

### ❌ "Ray pointing upward"
```yaml
# Adjust pitch limits in config.yaml
validation:
  min_pitch_deg: -60.0
  max_pitch_deg: 60.0
```

### ❌ Detections too far/close
```yaml
# Adjust FOV in config.yaml
camera:
  hfov_deg: 140.0  # Try ±5° adjustments
  vfov_deg: 78.0
```

### ❌ UDP not receiving
```bash
# Check firewall
sudo ufw allow 6000:6001/udp

# Verify Tailscale
tailscale status

# Ping test
ping 100.115.66.25
```

---

## Quick Reference Commands

```bash
# Complete workflow
python system_check.py                    # 1. Validate setup
python camera_calibration.py --capture    # 2. Calibrate camera
python camera_calibration.py --calibrate
python camera_calibration.py --update-config
python test_geotagging.py                 # 3. Test math
python manual_test.py                     # 4. Indoor test

# Live testing
# RPI:
python3 raspberry_pi_sender.py --laptop-ip 100.115.66.25 ...
# Laptop:
python maingeo.py --config config.yaml --mode live --show
```

---

## Understanding the Output

### Terminal Output
```
INFO - Frame #1: 2 detection(s)
INFO - New detection at (26.449123, 80.331456) - 5.2m away
```
- Frame count increases each detection
- GPS coordinates of detected person
- Distance from drone

### Map Output (`outputs/maps/`)
- **Blue plane icon** = Drone position
- **Red circles** = Detected people (unique)
- **Hover over** circles to see details
- Click to zoom in/out

### Image Output (`outputs/images/`)
- Green boxes = detections
- Label shows distance in meters

---

## Tuning Parameters (config.yaml)

**For more detections:**
```yaml
confidence_threshold: 0.15  # Lower = more detections
```

**For better accuracy:**
```yaml
max_geotag_distance: 100.0  # Only tag nearby objects
```

**For avoiding duplicates:**
```yaml
deduplication:
  min_distance_meters: 5.0   # Treat <5m as same person
  time_window_seconds: 60.0  # 60 sec memory window
```

**For pitch adjustment:**
```yaml
advanced:
  pitch_correction_deg: 5.0  # Add offset if consistently off
```

---

## Success Indicators ✅

**Indoor Test:**
- [ ] Camera captures person
- [ ] YOLO detects person (green box)
- [ ] Geotagging produces coordinates
- [ ] Map shows logical position
- [ ] Manual movement tests work correctly

**Live Hardware Test:**
- [ ] GPS has 3D fix
- [ ] Telemetry streaming to laptop
- [ ] Images arriving at laptop
- [ ] Detections occurring
- [ ] Maps being generated
- [ ] Distances approximately correct (±2-3m)

---

## Next Steps After Room Test

1. **Static outdoor test:** Place drone on ground, test at different distances
2. **Tethered hover test:** Hold drone while it hovers (someone else flies)
3. **Low altitude waypoint:** 5m altitude, simple square pattern
4. **Full flight test:** 20m altitude, cover search area

---

## Emergency Contacts

- Pixhawk docs: https://ardupilot.org/
- YOLO docs: https://docs.ultralytics.com/
- OpenCV calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

---

## File Structure

```
drone_geotagging/
├── config.yaml                 # Main config
├── maingeo.py                  # Laptop receiver
├── raspberry_pi_sender.py      # RPI sender
├── camera_calibration.py       # Calibration tool
├── manual_test.py              # Indoor test
├── test_geotagging.py          # Math validation
├── system_check.py             # Pre-flight check
├── outputs/
│   ├── images/                 # Annotated frames
│   └── maps/                   # Interactive maps
└── runs/train/.../weights/
    └── best.pt                 # Your YOLO model
```

Good luck! 🎯