# Drone Geotagging System - Setup Instructions

## Overview
This system consists of:
- **Raspberry Pi**: Captures images, reads Pixhawk telemetry, sends data via UDP
- **Laptop**: Receives data, runs YOLO detection, performs geotagging

Communication happens over **Tailscale VPN** for reliable connectivity.

---

## 1. Laptop Setup

### 1.1 Install Dependencies
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install required packages
pip install ultralytics opencv-python numpy folium pyyaml pymavlink
```

### 1.2 Install Tailscale
```bash
# Ubuntu/Debian
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# Get your Tailscale IP
tailscale ip -4
# Example output: 100.x.x.x
```

### 1.3 Configure the System
1. Edit `config.yaml`:
   - Set correct `model_path` to your YOLO model
   - Adjust FOV values if using different camera
   - Set appropriate validation thresholds

2. Test with sample data:
```bash
python drone_geotag.py \
  --config config.yaml \
  --mode test \
  --test-image /path/to/test/image.jpg \
  --test-telemetry test_telemetry.json
```

### 1.4 Run Live Mode
```bash
# Start listening for UDP data
python drone_geotag.py --config config.yaml --mode live
```

---

## 2. Raspberry Pi Setup

### 2.1 Install Dependencies
```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install Python packages
sudo apt-get install python3-pip python3-opencv
pip3 install pymavlink numpy

# Install Tailscale
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# Get your Tailscale IP
tailscale ip -4
```

### 2.2 Connect Hardware

#### Pixhawk Connection
- Connect Pixhawk TELEM2 port to Raspberry Pi UART (GPIO 14/15)
- OR use USB connection (`/dev/ttyUSB0`)

**Wiring (UART):**
```
Pixhawk TELEM2  →  Raspberry Pi
---------------------------------
TX (Pin 2)      →  RX (GPIO 15, Pin 10)
RX (Pin 3)      →  TX (GPIO 14, Pin 8)
GND (Pin 6)     →  GND (Pin 6)
```

**Enable UART:**
```bash
# Edit config
sudo nano /boot/config.txt

# Add these lines:
enable_uart=1
dtoverlay=disable-bt

# Reboot
sudo reboot
```

#### Camera Connection
- DJI Action 2: Connect via USB (appears as `/dev/video0`)
- Or use Raspberry Pi Camera Module

**Test camera:**
```bash
# List video devices
v4l2-ctl --list-devices

# Test capture
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Failed')"
```

### 2.3 Configure Pixhawk Parameters

Connect to Pixhawk via Mission Planner or QGroundControl:

**Required Parameters:**
```
# Serial port configuration (if using TELEM2)
SERIAL2_PROTOCOL = 2 (MAVLink2)
SERIAL2_BAUD = 57 (57600 baud)

# GPS configuration
GPS_TYPE = 1 (Auto)
EK3_SRC1_POSXY = 3 (GPS)
EK3_SRC1_POSZ = 1 (Baro)

# For accurate altitude
RNGFND1_TYPE = 1 (if using rangefinder)
```

### 2.4 Test Pixhawk Connection
```bash
# Test MAVLink connection
python3 << EOF
from pymavlink import mavutil
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Connected to system {master.target_system}")
EOF
```

### 2.5 Run Sender Script
```bash
# Replace with your laptop's Tailscale IP
python3 raspberry_pi_sender.py \
  --laptop-ip 100.x.x.x \
  --pixhawk-port /dev/ttyAMA0 \
  --baudrate 57600 \
  --camera-index 0 \
  --fps 2.0
```

### 2.6 Auto-start on Boot (Optional)
```bash
# Create systemd service
sudo nano /etc/systemd/system/drone-sender.service
```

Add:
```ini
[Unit]
Description=Drone Data Sender
After=network.target tailscaled.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/drone
ExecStart=/usr/bin/python3 /home/pi/drone/raspberry_pi_sender.py \
  --laptop-ip 100.x.x.x \
  --pixhawk-port /dev/ttyAMA0 \
  --baudrate 57600 \
  --camera-index 0 \
  --fps 2.0
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable drone-sender
sudo systemctl start drone-sender
sudo systemctl status drone-sender
```

---

## 3. Network Configuration

### 3.1 Tailscale Setup
Both devices should be on the same Tailscale network:

```bash
# On both devices, verify connection
tailscale status

# Test connectivity
# From laptop:
ping <raspberry-pi-tailscale-ip>

# From Raspberry Pi:
ping <laptop-tailscale-ip>
```

### 3.2 Firewall Rules
```bash
# On laptop, allow UDP ports
sudo ufw allow 5000:5001/udp

# Or disable firewall for Tailscale interface
sudo ufw allow in on tailscale0
```

### 3.3 Test UDP Connection
**On laptop:**
```bash
# Listen for UDP data
nc -ul 5000
```

**On Raspberry Pi:**
```bash
# Send test data
echo "test" | nc -u <laptop-tailscale-ip> 5000
```

---

## 4. Camera Calibration (Advanced)

For best accuracy with 143° FOV, calibrate camera distortion:

```python
import cv2
import numpy as np

# Use checkerboard calibration pattern
# Collect 20+ images of checkerboard from different angles
# Run OpenCV calibration

# Example:
objpoints = []  # 3D points
imgpoints = []  # 2D points

# ... calibration code ...

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_size, None, None
)

# Save to config.yaml
print("camera_matrix:", camera_matrix.tolist())
print("distortion_coeffs:", dist_coeffs.tolist())
```

---

## 5. Testing Procedure

### 5.1 Static Ground Test
1. Place drone on ground with known GPS coordinates
2. Place objects at measured distances
3. Run system and compare geotagged positions with actual
4. Verify accuracy within expected uncertainty

### 5.2 Flight Test Checklist
- [ ] Tailscale connection verified
- [ ] Pixhawk GPS lock (3D fix)
- [ ] Camera preview working
- [ ] Laptop receiving telemetry
- [ ] Laptop receiving images
- [ ] YOLO detection running
- [ ] Map generation working

### 5.3 Monitor Logs
**Laptop:**
```bash
tail -f drone_geotag.log
```

**Raspberry Pi:**
```bash
tail -f /home/pi/drone_sender.log
```

---

## 6. Troubleshooting

### No Telemetry Data
- Check Pixhawk serial connection
- Verify baud rate matches
- Check MAVLink parameters in Pixhawk
- Test with `mavproxy.py --master=/dev/ttyAMA0 --baudrate=57600`

### No Images
- Check camera connection: `ls /dev/video*`
- Test camera: `raspistill -o test.jpg` (Pi Camera) or `fswebcam test.jpg` (USB)
- Check permissions: `sudo usermod -a -G video pi`

### UDP Data Not Arriving
- Verify Tailscale IPs: `tailscale status`
- Check firewall: `sudo ufw status`
- Ping test between devices
- Check UDP ports not in use: `sudo netstat -ulnp | grep 5000`

### GPS Not Working
- Wait for GPS lock (can take 5-10 minutes)
- Check GPS antenna placement (clear sky view)
- Verify GPS type in Pixhawk parameters
- Check GPS status: `GPS_TYPE`, `GPS_STATUS`

### High Geotagging Errors
- Calibrate altitude (barometer + rangefinder)
- Calibrate compass
- Check gimbal pitch accuracy
- Perform camera calibration
- Verify coordinate rotation logic with known test points

### Low FPS
- Reduce image resolution
- Increase JPEG compression
- Reduce detection frequency
- Use smaller YOLO model (e.g., YOLOv8n)

---

## 7. Performance Optimization

### Raspberry Pi
```bash
# Overclock (edit /boot/config.txt)
arm_freq=1750
gpu_freq=600
over_voltage=6

# Disable GUI
sudo systemctl set-default multi-user.target
```

### Laptop
- Use GPU for YOLO inference
- Batch process frames if latency allows
- Reduce logging verbosity in production

---

## 8. Data Flow Diagram

```
┌─────────────────────┐
│   Pixhawk FC        │
│  - GPS/IMU/Baro     │
└──────────┬──────────┘
           │ MAVLink (Serial/USB)
           ▼
┌─────────────────────┐
│  Raspberry Pi       │
│  - Read telemetry   │
│  - Capture images   │
│  - Encode & send    │
└──────────┬──────────┘
           │ UDP over Tailscale
           ▼
┌─────────────────────┐
│     Laptop          │
│  - YOLO detection   │
│  - Geotagging       │
│  - Visualization    │
└─────────────────────┘
```

---

## 9. Safety Notes

⚠️ **Important:**
- Always maintain visual line of sight
- Follow local drone regulations
- Test thoroughly before flight
- Have backup manual control
- Monitor battery levels
- Check all connections before takeoff
- Start with low altitude tests (5-10m)

---

## 10. Next Steps

After basic setup:
1. Calibrate camera distortion
2. Add terrain elevation model
3. Implement multi-packet UDP for larger images
4. Add flight log recording
5. Create real-time dashboard
6. Implement object tracking across frames

---

## Support

For issues:
1. Check logs on both devices
2. Verify all connections
3. Test each component independently
4. Review MAVLink message rates
5. Monitor network latency

Good luck! 🚁