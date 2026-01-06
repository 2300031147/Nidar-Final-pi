# NIDAR Autonomous Drone System - Installation Guide

Complete installation guide for the fully autonomous human detection and delivery system.

---

## System Overview

**What This System Does** (100% Autonomous):
1. Scout drone detects humans with camera
2. GPS coordinates automatically marked
3. Delivery drone automatically dispatched
4. Package dropped at target location
5. Automatic return to launch and disarm

**Zero human interaction required!**

---

## Prerequisites

### Hardware Required

**Scout Drone**:
- Raspberry Pi 4 (4GB+ RAM recommended)
- USB Camera or Pi Camera Module
- Flight controller (Pixhawk/Cube)
- GPS module (6+ satellites required)

**Delivery Drone**:
- Raspberry Pi 4
- Flight controller (Pixhawk/Cube)
- GPS module (8+ satellites required)
- Servo for package drop (connected to AUX1)

**Backend Server**:
- Computer/Server with Docker
- Tailscale VPN configured

---

## Installation Steps

### 1. Scout Drone (Raspberry Pi)

```bash
# Navigate to project directory
cd ~/Nidar-Final-main/pi_scripts

# Install vision dependencies
pip3 install -r requirements_vision.txt

# This installs:
# - opencv-python-headless (computer vision)
# - numpy (array processing)
# - flask (video streaming)
```

**Connect Hardware**:
- USB camera to Raspberry Pi USB port
- Or Pi Camera to CSI port
- Ensure camera is detected: `ls /dev/video*`

**Configure Settings** (optional):
Edit `scout_tailscale.py`:
```python
# Camera resolution
CAMERA_WIDTH = 1280  # 720p
CAMERA_HEIGHT = 720

# Detection sensitivity
DETECTION_CONFIDENCE = 0.6  # Lower = more sensitive

# Streaming
STREAM_FPS = 24  # Frames per second
STREAM_PORT = 5001  # Video stream port
```

**Run Scout**:
```bash
python3 scout_tailscale.py
```

**Expected Output**:
```
✅ Heartbeat from System X, Component Y
📡 Starting video stream server on port 5001...
✅ Video stream available at http://0.0.0.0:5001/video_feed
🎥 Starting human detection loop...
✅ Human detection system started
```

---

### 2. Delivery Drone (Raspberry Pi)

```bash
cd ~/Nidar-Final-main/pi_scripts
python3 delivery_tailscale.py
```

**Hardware Setup**:
- Wire servo to **AUX1 (Channel 9)** on flight controller
- Test servo manually in QGroundControl first

**Configure Settings** (optional):
Edit `delivery_tailscale.py`:
```python
# Servo configuration
DROP_SERVO_CHANNEL = 9  # AUX1
DROP_SERVO_PWM_OPEN = 1900  # Release package
DROP_SERVO_PWM_CLOSE = 1100  # Hold package

# Drop parameters
DROP_DISTANCE_THRESHOLD = 2.0  # Meters from target
DROP_ALTITUDE = 3  # Drop height in meters
```

**Expected Output**:
```
✅ Heartbeat from System X, Component Y
📡 Requesting telemetry streams...
✅ Delivery Online
```

---

### 3. Backend Server

```bash
cd ~/Nidar-Final-main

# Build and start services
docker-compose up --build -d

# Check logs
docker-compose logs -f backend
```

**Configure Auto-Dispatch** (optional):
Edit `backend/server.py`:
```python
# Fully autonomous (default)
AUTO_DISPATCH_ENABLED = True

# Require manual button click
AUTO_DISPATCH_ENABLED = False

# Cooldown between auto-dispatches
DISPATCH_COOLDOWN = 60  # Seconds
```

**Expected Output**:
```
✅ API Connected to Broker
📡 Subscribed to telemetry and detection topics
 * Running on all addresses (0.0.0.0)
 * Running on http://127.0.0.1:5000
```

---

### 4. Frontend

```bash
# Already built with docker-compose
# Access at: http://localhost:3000
# Or: http://<server-ip>:3000
```

**Features**:
- 🗺️ Map view with drone tracking
- 📹 Live camera feed page (720p @ 24fps)
- Real-time detection markers
- Drone telemetry and control
- Auto-dispatch status

---

## Configuration Options

### Scout Drone

**Disable Auto-Detection**:
```python
CAMERA_ENABLED = False  # In scout_tailscale.py
```

**Adjust Detection Rules**:
```python
DETECTION_COOLDOWN = 60  # Increase time between detections
DETECTION_MIN_DISTANCE = 30  # Increase distance filter
DETECTION_CONFIDENCE = 0.4  # More sensitive (0.8 = less sensitive)
```

**Disable Video Streaming**:
```python
STREAM_ENABLED = False
```

---

### Delivery Drone

**Disable Auto-Disarm**:
```python
AUTO_DISARM_ENABLED = False  # In delivery_tailscale.py
```

**Adjust Drop Precision**:
```python
DROP_DISTANCE_THRESHOLD = 3.0  # Drop at 3m instead of 2m
DROP_ALTITUDE = 5  # Drop from 5m height
```

---

### Backend

**Disable Auto-Dispatch**:
```python
AUTO_DISPATCH_ENABLED = False  # In server.py
```
Requires manual "Dispatch Delivery" button click.

**Adjust Dispatch Cooldown**:
```python
DISPATCH_COOLDOWN = 120  # 2 minutes between dispatches
```

---

## Quick Test

### Test 1: Camera Feed
```bash
# Check video stream directly
curl http://<scout-ip>:5001/video_feed

# Or open in browser
http://<scout-ip>:5001/video_feed
```

### Test 2: Detection System
```bash
# Run test script on scout Pi
python3 test_detection.py

# Walk in front of camera
# Check for bounding boxes in output image
```

### Test 3: Full Autonomous Cycle
1. Start all components (scout, delivery, backend, frontend)
2. Arm scout drone and fly
3. Walk in front of scout camera
4. Watch system:
   - Detection logged
   - Auto-dispatch triggered
   - Delivery mission executes
   - Package dropped
   - Auto-return and disarm

---

## Troubleshooting

### Camera Not Working

**Check camera**:
```bash
ls /dev/video*  # Should show /dev/video0
v4l2-ctl --list-devices  # List all cameras
```

**Enable camera** (Pi Camera):
```bash
sudo raspi-config
# Interface Options → Camera → Enable
sudo reboot
```

### No Detections Happening

**Check GPS lock**:
- Scout needs **6+ satellites**
- Delivery needs **8+ satellites**

**Lower sensitivity**:
```python
DETECTION_CONFIDENCE = 0.4  # More sensitive
```

**Check logs**:
```
❌ Detection REJECTED - No valid GPS lock
⚠️ GPS Lock Check Failed: Sats=4, Fix=2
```

### Auto-Dispatch Not Working

**Verify configuration**:
```python
# In backend/server.py
AUTO_DISPATCH_ENABLED = True  # Must be True
```

**Check cooldown**:
```
⏱️ Auto-dispatch blocked by cooldown (15.2s < 60s)
```
Wait 60 seconds between detections.

**Check backend logs**:
```bash
docker-compose logs -f backend
# Should show: "🚁 AUTO-DISPATCH: Delivery sent to..."
```

### Auto-Disarm Not Working

**Verify RTL landing**:
- Drone must complete RTL (Return to Launch)
- Altitude must drop below 0.5m
- Waits 3 seconds then disarms

**Check logs**:
```
✅ Landed (alt=0.3m) - Auto-disarming in 3 seconds...
✅ Auto-disarm complete
```

---

## System Requirements

### GPS Requirements

**Scout Drone**:
- Minimum **6 satellites**
- GPS Fix Type **3** (3D fix)
- Clear sky view

**Delivery Drone**:
- Minimum **8 satellites** (stricter for precision)
- GPS Fix Type **3**
- Clear sky view

### Network Requirements

- All devices on same Tailscale network
- Ports required:
  - **5000**: Backend API/MQTT
  - **5001**: Scout camera stream
  - **3000**: Frontend web UI
  - **1883**: MQTT broker

### Performance

**Raspberry Pi 4**:
- Scout: ~30-40% CPU (streaming + detection)
- Delivery: ~10-20% CPU

**Bandwidth**:
- Camera stream: ~1-2 Mbps
- MQTT telemetry: <10 Kbps

---

## Safety Checklist

Before flying:
- [ ] GPS lock achieved (check satellite count)
- [ ] Camera feed visible on website
- [ ] Detection system active
- [ ] Servo tested (delivery drone)
- [ ] Flight area clear of obstacles
- [ ] Emergency DISARM button accessible
- [ ] RTL home position set correctly

---

## Feature Summary

✅ **Human Detection**: OpenCV HOG with 720p camera  
✅ **GPS Validation**: 6+ sats (scout), 8+ sats (delivery)  
✅ **Duplicate Prevention**: Time + distance + exact location filters  
✅ **Auto-Dispatch**: Immediate delivery on detection  
✅ **Precision Navigation**: 2m accuracy with GPS monitoring  
✅ **Single Drop Protection**: One package per mission  
✅ **Auto-Disarm**: Safe shutdown after landing  
✅ **Live Camera Feed**: 720p @ 24fps MJPEG stream  
✅ **Detection Overlay**: Green bounding boxes on video  
✅ **Dual View UI**: Map + dedicated camera page  

---

## Next Steps

1. **Install dependencies** on both Pis
2. **Test camera** with `test_detection.py`
3. **Test servo** in QGroundControl
4. **Start all services**
5. **Verify GPS locks**
6. **Test in safe area**
7. **Monitor logs** during operation

---

## Support Documentation

- **AUTONOMOUS_OPERATION.md**: Full autonomous cycle details
- **CAMERA_FEED.md**: Video streaming configuration
- **DUPLICATE_DETECTION.md**: Detection filtering rules
- **DELIVERY_GPS_VALIDATION.md**: GPS requirements
- **SINGLE_DROP_PROTECTION.md**: Drop safety mechanism
- **walkthrough.md**: Complete system walkthrough

---

**System is production-ready for autonomous rescue operations!**
