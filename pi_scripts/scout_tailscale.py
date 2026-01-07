from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json
import time
import threading
import cv2
import numpy as np
from datetime import datetime
import math
from flask import Flask, Response
# from flask_cors import CORS
from ultralytics import YOLO # pip install ultralytics
# 🔴 CONFIGURATION - YOUR TAILSCALE IP
BROKER_IP = "100.125.45.22"
BROKER_SECONDARY_IP = "100.102.90.88" # Backup/Secondary Laptop

TOPIC_MISSION = "nidar/scout/mission"
TOPIC_TELEM = "nidar/scout/telemetry"
TOPIC_SURVIVOR = "nidar/delivery/target"
TOPIC_DETECTIONS = "nidar/scout/detections"  # Human detection topic

# CAMERA & DETECTION SETTINGS
CAMERA_ENABLED = True  # Set to False to disable detection
CAMERA_INDEX = 0  # USB camera index (0 = /dev/video0)
CAMERA_WIDTH = 1280  # 720p width
CAMERA_HEIGHT = 720  # 720p height
DETECTION_FPS = 2.0  # Check for humans 2 times per second (Allows persistence check)
DETECTION_CONFIDENCE = 0.0  # Confidence threshold for detections (Tuned for sitting/indoor)
DETECTION_COOLDOWN = 30  # Seconds between detections at same location
DETECTION_MIN_DISTANCE = 15  # Minimum meters between detection points

# VIDEO STREAMING SETTINGS
STREAM_ENABLED = True  # Enable MJPEG streaming
STREAM_PORT = 5001  # Flask server port
STREAM_FPS = 15  # Stream frames per second (Reduced for bandwidth)
STREAM_QUALITY = 30  # JPEG quality (1-100) (Lowered to reduce bitrate)

# GEOLOCATION CONFIGURATION
CAMERA_FOV_H = 62.2  # Horizontal Field of View (degrees) - Standard for RPi Cam V2
CAMERA_FOV_V = 48.8  # Vertical Field of View (degrees)
CAMERA_PITCH_DEG = -70.0  # Camera pitch angle (0=forward, -90=down) 
# Note: Ensure this matches your physical mount angle!

def get_target_gps(drone_lat, drone_lon, alt, heading_deg, center_x, center_y, img_w, img_h):
    """
    Calculate target GPS based on drone position/orientation and pixel offset.
    """
    if alt <= 0: return drone_lat, drone_lon
    
    # avoid division by zero
    if img_w == 0 or img_h == 0: return drone_lat, drone_lon

    # 1. Calculate Angular Offsets from Center
    # Normalized coordinates (-1 to 1)
    norm_x = (center_x - img_w/2.0) / (img_w/2.0)
    norm_y = (center_y - img_h/2.0) / (img_h/2.0)
    
    # Angle offsets
    angle_x = norm_x * (CAMERA_FOV_H / 2.0)
    angle_y = -norm_y * (CAMERA_FOV_V / 2.0) # Inverted Y (pixel y goes down)
    
    # 2. Estimate Ground Distance
    # Effective pitch to the target pixel
    pixel_pitch = CAMERA_PITCH_DEG + angle_y
    
    # If looking above horizon, can't intersect ground
    if pixel_pitch >= 0:
        dist_ground = 500.0 # Cap at max range
    else:
        # tan(pitch) = height / dist -> dist = height / tan(abs(pitch))
        dist_ground = alt / math.tan(math.radians(abs(pixel_pitch)))
    
    # Cap insane distances
    if dist_ground > 200: dist_ground = 200
        
    # 3. Calculate Bearing to Target
    bearing_deg = (heading_deg + angle_x) % 360
    
    # 4. Project New Coordinates using Haversine destination
    # lat/lon in radians
    R = 6371000.0
    lat1 = math.radians(drone_lat)
    lon1 = math.radians(drone_lon)
    bearing = math.radians(bearing_deg)
    
    lat2 = math.asin(math.sin(lat1)*math.cos(dist_ground/R) +
                     math.cos(lat1)*math.sin(dist_ground/R)*math.cos(bearing))
    
    lon2 = lon1 + math.atan2(math.sin(bearing)*math.sin(dist_ground/R)*math.cos(lat1),
                             math.cos(dist_ground/R)-math.sin(lat1)*math.sin(lat2))
                             
    return math.degrees(lat2), math.degrees(lon2)

COPTER_MODES = {
    0: 'STABILIZE', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 9: 'LAND'
}

print("🔌 Connecting to Mavlink Router (UDP)...")
# Use 0.0.0.0 to listen on all interfaces (Localhost + Tailscale/LAN)
drone = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

print("⏳ Waiting for heartbeat...")
while True:
    msg = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        drone.target_system = msg.get_srcSystem()
        drone.target_component = msg.get_srcComponent()
        print(f"✅ Heartbeat from System {drone.target_system}, Component {drone.target_component}")
        break
    print("⚠️ No heartbeat yet... check Mavlink Router is running and sending to port 14550")

print("✅ Scout Online")

# Request telemetry streams from the flight controller
print("📡 Requesting telemetry streams...")
drone.mav.request_data_stream_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4, 1)  # 4Hz rate, enable=1

import subprocess

def get_drone_ip():
    """Detect drone IP (prefer Tailscale)."""
    try:
        return subprocess.check_output(["tailscale", "ip", "-4"], text=True).strip()
    except:
        try:
            # Fallback to first non-loopback IP
            cmd = "hostname -I | awk '{print $1}'"
            return subprocess.check_output(cmd, shell=True, text=True).strip()
        except:
            return "127.0.0.1"

# --- STATE ---
DRONE_DATA = {
    "lat": 0, "lon": 0, "alt": 0, 
    "bat": 0,           # Battery percentage (0-100)
    "bat_voltage": 0,   # Voltage in volts
    "bat_current": 0,   # Current draw in amps
    "bat_time_min": 0,  # Estimated minutes remaining
    "gps_sats": 0,      # Number of GPS satellites
    "status": "DISARMED", "speed": 0, "mode": "UNKNOWN",
    "detections_count": 0,  # Total humans detected
    "ip": get_drone_ip()    # Auto-detected IP
}
print(f"🌍 DRONE IP DETECTED: {DRONE_DATA['ip']}")
BATTERY_CAPACITY_MAH = 5000  # Default battery capacity, adjust for your battery
LAST_HEARTBEAT = 0
MISSION_UPLOAD_IN_PROGRESS = False  # Track mission upload state
AUTO_START_MISSION = False  # Flag to auto-start mission after upload
MISSION_POINTS = [] # Store mission to respond to FC requests

# DETECTION STATE
DETECTIONS = []  # List of detected humans: [{"lat": X, "lon": Y, "timestamp": T, "id": unique_id}]
LAST_DETECTION_TIME = 0
CAMERA = None
HOG_DETECTOR = None
DETECTION_ID_COUNTER = 0  # Unique ID for each detection

# VIDEO STREAMING STATE
latest_frame = None  # Latest camera frame for streaming
latest_detections = []  # Latest detection boxes for overlay
frame_lock = threading.Lock()  # Thread-safe frame access
stream_app = Flask('camera_stream')  # Flask app for streaming

@stream_app.after_request
def add_cors(response):
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Authorization')
    response.headers.add('Access-Control-Allow-Methods', 'GET,PUT,POST,DELETE,OPTIONS')
    return response

# Human-readable mapping for MAV_CMD_ACK results
MAV_RESULT_NAMES = {
    mavutil.mavlink.MAV_RESULT_ACCEPTED: "Accepted",
    mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: "Temp Rejected",
    mavutil.mavlink.MAV_RESULT_DENIED: "Denied",
    mavutil.mavlink.MAV_RESULT_UNSUPPORTED: "Unsupported",
    mavutil.mavlink.MAV_RESULT_FAILED: "Failed",
    mavutil.mavlink.MAV_RESULT_IN_PROGRESS: "In Progress",
    getattr(mavutil.mavlink, 'MAV_RESULT_CANCELLED', 6): "Cancelled"
}

# --- COMMAND HELPERS (Send Only) ---
def send_arm():
    print("⚠️ Sending ARM Command...")
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    # Do NOT set mode immediately after arm; let it complete first 

def send_disarm(force=True):
    """Disarm the drone. Uses force=True by default to bypass safety checks."""
    print("⚠️ Sending DISARM Command...")
    # ArduPilot magic number 21196 forces disarm bypassing safety checks
    magic_override = 21196 if force else 0
    
    # If not forcing, switch to LAND mode first
    if not force and DRONE_DATA.get('mode') != 'LAND':
        print("⚠️ Switching to LAND mode before DISARM...")
        drone.set_mode('LAND')
        time.sleep(2)  # Wait for landing to initiate
    
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, magic_override, 0, 0, 0, 0, 0)

# Global flag for Indoor Mode
INDOOR_MODE = False

def set_indoor_mode():
    global INDOOR_MODE
    INDOOR_MODE = True
    print("⚠️ SETTING INDOOR MODE (Safety Checks Disabled + GPS Bypass)")
    # Disable Arming Checks
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Disable Radio Failsafe (FS_THR_ENABLE)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'FS_THR_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Disable GCS Failsafe (FS_GCS_ENABLE)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'FS_GCS_ENABLE', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Disable Auto Disarm Delay (DISARM_DELAY)
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'DISARM_DELAY', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    # Ensure motor spin
    drone.mav.param_set_send(drone.target_system, drone.target_component, b'MOT_SPIN_ARM', 0.10, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

def override_rc(channels):
    pwm = [65535] * 8
    for i, val in enumerate(channels):
        if i < 8: pwm[i] = int(val)
    print(f"🎮 RC OVERRIDE: {pwm}")
    drone.mav.rc_channels_override_send(drone.target_system, drone.target_component, *pwm)

def upload_mission_to_fc(waypoints, alt=20, speed=None, auto_start=True):
    global MISSION_POINTS, MISSION_UPLOAD_IN_PROGRESS, AUTO_START_MISSION
    print(f"📤 Uploading {len(waypoints)} points (Alt: {alt}m, Speed: {speed})...")
    
    # Construct Mission List
    # If speed is set, add DO_CHANGE_SPEED command first
    mission_items = []
    
    if speed and speed > 0:
        mission_items.append({
            'type': 'SPEED',
            'speed': speed
        })
        
    # Add waypoints
    for wp in waypoints:
        mission_items.append({
            'type': 'WAYPOINT',
            'lat': wp['lat'],
            'lng': wp['lng'],
            'alt': alt
        })
        
    MISSION_POINTS = mission_items # Store for Protocol
    MISSION_UPLOAD_IN_PROGRESS = True
    AUTO_START_MISSION = auto_start  # Store whether to auto-start after upload
    drone.mav.mission_clear_all_send(drone.target_system, drone.target_component)
    time.sleep(0.2)  # Small delay for clear to process
    # Send Count - FC will then request each Wpoint
    drone.mav.mission_count_send(drone.target_system, drone.target_component, len(mission_items))

def send_mission_item(seq):
    if seq < len(MISSION_POINTS):
        item = MISSION_POINTS[seq]
        
        if item['type'] == 'SPEED':
            print(f"   sending SPEED change: {item['speed']} m/s")
            drone.mav.mission_item_int_send(
                drone.target_system, drone.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0, 1, # current, autocontinue
                0, item['speed'], -1, 0, # param1=0(groundspeed), param2=speed, param3=-1(no throttle limit), param4=0
                0, 0, 0 # x,y,z unused
            )
        elif item['type'] == 'WAYPOINT':
            print(f"   sending WP {seq}: {item['lat']}, {item['lng']} Alt:{item['alt']}")
            drone.mav.mission_item_int_send(
                drone.target_system, drone.target_component,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 1, # current, autocontinue
                0, 0, 0, 0, # params 1-4
                int(item['lat'] * 1e7),
                int(item['lng'] * 1e7),
                int(item['alt']) # Altitude from mission settings
            )

def handle_takeoff_command(alt):
    """Handle TAKEOFF command with proper sequencing (runs in separate thread)."""
    try:
        # Step 1: Validate GPS position
        print(f"🛫 TAKEOFF sequence starting (alt={alt}m)...")
        
        # Check for GPS, but allow bypass if INDOOR_MODE is True
        for _ in range(10):
            if DRONE_DATA.get('lat', 0) != 0 and DRONE_DATA.get('lon', 0) != 0:
                break
            time.sleep(0.1)
        
        lat = float(DRONE_DATA.get('lat', 0))
        lon = float(DRONE_DATA.get('lon', 0))
        
        if (lat == 0 or lon == 0) and not INDOOR_MODE:
            print("❌ TAKEOFF aborted: No valid GPS position (lat/lon=0)")
            print("   💡 If strictly necessary, enable 'Indoor Mode' to bypass this check.")
            return
        
        if INDOOR_MODE and (lat == 0 or lon == 0):
             print("⚠️ Indoor Mode: Bypassing GPS check...")
        
        # Step 2: Ensure GUIDED mode (CRITICAL - TAKEOFF only works in GUIDED)
        current_mode = DRONE_DATA.get('mode', '')
        if current_mode != 'GUIDED':
            print(f"⚠️ Switching from {current_mode} to GUIDED mode...")
            drone.set_mode('GUIDED')
            time.sleep(1.5)  # Wait for mode change
        
        # Step 3: Ensure armed
        if DRONE_DATA.get('status') != 'ARMED':
            print("⚠️ Not armed; arming before takeoff...")
            send_arm()
            time.sleep(2)  # Wait for arming to complete
        
        # Step 4: Send TAKEOFF command
        print(f"🛫 Sending TAKEOFF to {alt}m at ({lat:.6f}, {lon:.6f})")
        drone.mav.command_long_send(
            drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,              # confirmation
            0,              # param1: minimum pitch (if airspeed sensor)
            0,              # param2: empty
            0,              # param3: empty
            float('nan'),   # param4: yaw (NaN = use current heading)
            lat,            # param5: latitude
            lon,            # param6: longitude
            alt)            # param7: altitude
    except Exception as e:
        print(f"❌ TAKEOFF Error: {e}")

# --- MQTT CALLBACKS ---
def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        print(f"📩 MQTT RAW: {payload}")
        data = json.loads(payload)
        
        if data['type'] == "COMMAND":
            act = data.get('act')
            print(f"⚙️ EXECUTING COMMAND: {act}")
            
            if act == "ARM": 
                send_arm()
            elif act == "DISARM":
                force = bool(data.get('force', True))  # Default to force disarm
                send_disarm(force=force)
            elif act == "LAND": 
                drone.set_mode('LAND')
                print("🛬 LANDING")
            elif act == "TAKEOFF":
                # Use threading to avoid blocking MQTT callback
                alt = float(data.get('alt', 10))
                threading.Thread(target=handle_takeoff_command, args=(alt,), daemon=True).start()
            elif act == "INDOOR_MODE": 
                set_indoor_mode()
            elif act == "RC_OVERRIDE": 
                override_rc(data.get('channels', []))
            elif act == "SET_MODE":
                mode = data.get('mode')
                print(f"⚠️ Mode -> {mode}")
                drone.set_mode(mode)
        elif data['type'] == "MISSION_UPLOAD":
            waypoints = data.get('waypoints', data.get('points')) # Support both keys
            alt = data.get('alt', 20)
            speed = data.get('speed', 5)
            upload_mission_to_fc(waypoints, alt=alt, speed=speed)

    except Exception as e:
        print(f"❌ MQTT Message Error: {e}")
        # import traceback
        # traceback.print_exc()

client1 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="scout_primary")
client2 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="scout_secondary")

def on_connect_factory(name):
    def on_connect(client, userdata, flags, rc, properties=None):
        if rc == 0:
            print(f"✅ Connected to {name} MQTT Broker")
            client.subscribe(TOPIC_MISSION)
            print(f"📡 Subscribed to {TOPIC_MISSION} on {name}")
        else:
            print(f"❌ Connection to {name} Failed: Code {rc}")
    return on_connect

client1.on_connect = on_connect_factory("Primary")
client1.on_message = on_message
client2.on_connect = on_connect_factory("Secondary")
client2.on_message = on_message

def mqtt_loop_thread(client, ip, name):
    print(f"⏳ Connecting to {name} Broker ({ip})...")
    while True:
        try:
            client.connect(ip, 1883, 60)
            client.loop_forever() # Use loop_forever for dedicated threads
        except Exception as e:
            print(f"⚠️ {name} MQTT Connection Failed: {e}")
            time.sleep(5)

# Start MQTT threads
threading.Thread(target=mqtt_loop_thread, args=(client1, BROKER_IP, "Primary"), daemon=True).start()
threading.Thread(target=mqtt_loop_thread, args=(client2, BROKER_SECONDARY_IP, "Secondary"), daemon=True).start()

# --- HUMAN DETECTION FUNCTIONS ---
def init_camera():
    """Initialize camera and HOG detector."""
    global CAMERA, HOG_DETECTOR
    try:
        print("📷 Initializing camera...")
        # Use CAP_V4L2 for Raspberry Pi (Linux)
        CAMERA = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not CAMERA.isOpened():
            print("❌ Failed to open camera (V4L2)")
            return False
        
        # CRITICAL: Force MJPEG format to avoid USB bandwidth issues at 720p
        CAMERA.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        CAMERA.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        CAMERA.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        CAMERA.set(cv2.CAP_PROP_FPS, STREAM_FPS)
        
        # Initialize YOLOv8 for human detection
        # Using 'yolov8n.pt' (Nano) for Raspberry Pi performance
        print("🧠 Loading YOLOv8 Nano model...")
        HOG_DETECTOR = YOLO('yolov8n.pt') 
        # Note: We keep the variable name HOG_DETECTOR to minimize refactoring, 
        # but it is now a YOLO model instance.
        
        print("✅ Camera and YOLO initialized successfully")
        return True
    except Exception as e:
        print(f"❌ Camera initialization error: {e}")
        # Clean up on error
        if CAMERA is not None:
            CAMERA.release()
        return False

def calculate_distance(lat1, lon1, lat2, lon2):
    """Calculate distance in meters between two GPS coordinates using Haversine formula."""
    R = 6371000  # Earth radius in meters
    
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    
    a = math.sin(delta_lat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def has_gps_lock():
    """Check if drone has valid GPS lock."""
    gps_sats = DRONE_DATA.get('gps_sats', 0)
    gps_fix = DRONE_DATA.get('gps_fix', 0)
    lat = DRONE_DATA.get('lat', 0)
    lon = DRONE_DATA.get('lon', 0)
    
    # Require:
    # - At least 6 satellites (minimum for 3D fix)
    # - GPS fix type 3 or higher (3D fix)
    # - Non-zero coordinates
    # - Coordinates not at default location
    
    has_lock = (
        gps_sats >= 6 and
        gps_fix >= 3 and
        lat != 0 and lon != 0 and
        lat != 16.506  # Not default location
    )
    
    if not has_lock:
        print(f"⚠️ GPS Lock Check Failed: Sats={gps_sats}, Fix={gps_fix}, Pos=({lat:.6f}, {lon:.6f})")
    
    return has_lock

def is_duplicate_detection(lat, lon):
    """Check if this location was recently detected - strict filtering to prevent duplicates."""
    global DETECTIONS, LAST_DETECTION_TIME
    
    # Rule 1: Global cooldown - minimum time between ANY detections
    time_since_last = time.time() - LAST_DETECTION_TIME
    if time_since_last < DETECTION_COOLDOWN:
        print(f"⏱️ Duplicate blocked by cooldown ({time_since_last:.1f}s < {DETECTION_COOLDOWN}s)")
        return True
    
    # Rule 2: Distance check - ensure new detection is far enough from ALL previous detections
    for detection in DETECTIONS:
        dist = calculate_distance(lat, lon, detection['lat'], detection['lon'])
        if dist < DETECTION_MIN_DISTANCE:
            print(f"📏 Duplicate blocked by distance ({dist:.1f}m < {DETECTION_MIN_DISTANCE}m) - Detection #{detection.get('id', '?')}")
            return True
    
    # Rule 3: Exact location check - prevent same GPS coordinate being detected twice
    for detection in DETECTIONS:
        if abs(detection['lat'] - lat) < 0.00001 and abs(detection['lon'] - lon) < 0.00001:
            print(f"🎯 Duplicate blocked by exact location match - Detection #{detection.get('id', '?')}")
            return True
    
    return False

def detect_humans(frame):
    """Detect humans in frame using YOLOv8."""
    if HOG_DETECTOR is None:
        return []
    
    try:
        # Resize NOT needed for YOLO (it handles it internally), 
        # but passing smaller frame saves inference time? 
        # YOLOv8n expects 640x640 usually. 
        # We'll pass the full frame but use a confidence threshold.
        
        # Run Inference
        # classes=[0] enforces only 'person' class detection
        # conf=0.4 is a good balance for humans
        # imgsz=416: Optimization for Pi 4 (Balance of Speed vs 20ft Detection Range)
        # half=False: Pi 4 CPU does not support FP16 math well
        results = HOG_DETECTOR(frame, classes=[0], conf=0.4, imgsz=416, half=False, verbose=False)
        
        detections = []
        for r in results:
            for box in r.boxes:
                # Get box coordinates (x1, y1, x2, y2)
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                # Get confidence
                score = float(box.conf[0])
                
                # Calculate width and height
                w = x2 - x1
                h = y2 - y1
                
                # Aspect Ratio Filter (Optional, but YOLO is usually smarter)
                # Keep it loose just in case
                # aspect_ratio = h / float(w)
                
                # DEBUG Log
                print(f"🧠 YOLO: Person Detected! Score={score:.2f}, Box={w}x{h}")
                
                detections.append((x1, y1, w, h, score))
        
        return detections
    except Exception as e:
        print(f"❌ YOLO Detection error: {e}")
        return []

def publish_detection(lat, lon):
    """Publish human detection to MQTT."""
    detection_data = {
        "lat": lat,
        "lon": lon,
        "timestamp": datetime.now().isoformat(),
        "drone": "scout",
        "alt": DRONE_DATA.get('alt', 0)
    }
    
    try:
        client1.publish(TOPIC_DETECTIONS, json.dumps(detection_data))
        client2.publish(TOPIC_DETECTIONS, json.dumps(detection_data))
        print(f"🚨 HUMAN DETECTED at ({lat:.6f}, {lon:.6f}) - Published to MQTT (Both Brokers)")
    except Exception as e:
        print(f"❌ Failed to publish detection: {e}")

def mark_detection():
    """Record detection with current GPS coordinates."""
    global DETECTIONS, LAST_DETECTION_TIME, DETECTION_ID_COUNTER
    
    # CRITICAL: Verify GPS lock before marking detection
    # BYPASS FOR INDOOR TESTING: If we found a human, we want to know!
    # We will use 0,0 or a fake location if no GPS.
    if not has_gps_lock():
        print("⚠️ No GPS Lock, but marking detection for INDOOR TEST.")
        # Proceed anyway...
        
    lat = DRONE_DATA.get('lat', 0)
    lon = DRONE_DATA.get('lon', 0)
    gps_sats = DRONE_DATA.get('gps_sats', 0)
    gps_fix = DRONE_DATA.get('gps_fix', 0)
    
    # Check for duplicates with strict filtering
    if is_duplicate_detection(lat, lon):
        print("⚠️ Detection REJECTED - Duplicate location or too soon")
        return False
    
    # Calculate Target Location using Geolocation Logic
    head = DRONE_DATA.get('heading', 0)
    alt = DRONE_DATA.get('alt', 0)
    
    target_lat, target_lon = get_target_gps(lat, lon, alt, head, CAMERA_WIDTH/2, CAMERA_HEIGHT/2, CAMERA_WIDTH, CAMERA_HEIGHT)
    
    # Generate unique ID for this detection
    DETECTION_ID_COUNTER += 1
    detection_id = DETECTION_ID_COUNTER
    
    # Record detection with metadata
    detection = {
        "id": detection_id,
        "lat": target_lat, # Use PROJECTED lat
        "lon": target_lon, # Use PROJECTED lon
        "drone_lat": lat,  # Keep original too
        "drone_lon": lon,
        "timestamp": time.time(),
        "alt": alt,
        "gps_sats": gps_sats,
        "gps_fix": gps_fix,
        "detection_time": datetime.now().isoformat()
    }
    DETECTIONS.append(detection)
    DRONE_DATA["detections_count"] = len(DETECTIONS)
    LAST_DETECTION_TIME = time.time()
    
    # Publish to MQTT
    print(f"✅ DETECTION #{detection_id} MARKED - GPS Lock: {gps_sats} sats, Fix Type {gps_fix}")
    print(f"   Drone: ({lat:.6f}, {lon:.6f}) -> Target: ({target_lat:.6f}, {target_lon:.6f})")
    publish_detection(target_lat, target_lon)
    
    return True

def camera_loop():
    """Main camera processing loop (runs in separate thread)."""
    global CAMERA, latest_frame, latest_detections
    
    if not init_camera():
        print("❌ Camera disabled - initialization failed")
        return
    
    print("🎥 Starting human detection loop...")
    frame_interval = 1.0 / DETECTION_FPS
    last_process_time = 0
    
    # Persistence counter to filter noise
    # We require consecutive detections to confirm a human
    consecutive_detections = 0
    REQUIRED_CONSECUTIVE = 2  # Must detect in 2 frames in a row

    while True:
        try:
            # Capture frame for both detection and streaming
            if CAMERA is None or not CAMERA.isOpened():
                print("⚠️ Camera disconnected, attempting to reconnect...")
                if init_camera():
                    consecutive_errors = 0
                else:
                    time.sleep(2)
                    continue

            ret, frame = CAMERA.read()
            if not ret:
                print("⚠️ Failed to capture frame")
                consecutive_errors += 1
                
                # If too many failures, force re-init
                if consecutive_errors > 5:
                    print("❌ Too many frame failures, resetting camera...")
                    CAMERA.release()
                    consecutive_errors = 0
                    
                time.sleep(1)
                continue
            
            # Reset error count on success
            consecutive_errors = 0
            
            # Update streaming frame (always, for live feed)
            with frame_lock:
                latest_frame = frame.copy()
            
            # Process for detection at reduced rate
            if time.time() - last_process_time < frame_interval:
                time.sleep(0.05) # FAST sleep to check again soon
                continue
            
            # Detect humans
            detections = detect_humans(frame)
            
            # Update detection boxes for overlay
            with frame_lock:
                latest_detections = detections
            
            if len(detections) > 0:
                consecutive_detections += 1
                print(f"� Potential Detection ({consecutive_detections}/{REQUIRED_CONSECUTIVE})... Scores: {[round(d[4], 2) for d in detections]}")
                
                if consecutive_detections >= REQUIRED_CONSECUTIVE:
                    print(f"👤 CONFIRMED Human Detection!")
                    # Mark detection (will filter duplicates internally)
                    mark_detection()
                    # Reset counter slightly to allow re-triggering but don't spam
                    consecutive_detections = 1 
            else:
                if consecutive_detections > 0:
                    print(f"🗑️ Detection lost (Noise?)")
                consecutive_detections = 0
            
            last_process_time = time.time()
            
        except Exception as e:
            print(f"❌ Camera loop error: {e}")
            consecutive_errors += 1
            time.sleep(1)


# Start camera thread if enabled
if CAMERA_ENABLED:
    camera_thread = threading.Thread(target=camera_loop, daemon=True)
    camera_thread.start()
    print("✅ Human detection system started")
else:
    print("⚠️ Camera/Detection disabled in config")

# --- VIDEO STREAMING SERVER ---
@stream_app.route('/video_feed')
def video_feed():
    """MJPEG streaming endpoint."""
    print(f"📡 Video Feed Requested by Client!")
    return Response(generate_frames(), 
                   mimetype='multipart/x-mixed-replace; boundary=frame')

def generate_frames():
    """Generate MJPEG stream with detection overlay."""
    frame_time = 1.0 / STREAM_FPS
    
    while True:
        start_time = time.time()
        
        # Get latest frame and detections
        with frame_lock:
            if latest_frame is None:
                # No frame yet, send blank
                blank = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting for camera...", (50, CAMERA_HEIGHT//2), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                frame = blank
            else:
                frame = latest_frame.copy()
                detections = latest_detections.copy()
                
                # Draw detection boxes
                for (x, y, w, h, weight) in detections:
                    # Green bounding box
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 3)
                    # Confidence score
                    label = f"Human {weight:.2f}"
                    cv2.putText(frame, label, (x, y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Add status overlay
                gps_sats = DRONE_DATA.get('gps_sats', 0)
                detections_count = DRONE_DATA.get('detections_count', 0)
                status = f"GPS: {gps_sats} sats | Detections: {detections_count}"
                cv2.putText(frame, status, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, status, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
        
        # Encode to JPEG
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, STREAM_QUALITY])
        if not ret:
            time.sleep(0.1)
            continue
        
        frame_bytes = buffer.tobytes()
        
        # Yield frame in MJPEG format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        # Maintain target FPS
        elapsed = time.time() - start_time
        if elapsed < frame_time:
            time.sleep(frame_time - elapsed)

# Start Flask streaming server
if STREAM_ENABLED and CAMERA_ENABLED:
    def run_stream_server():
        print(f"📡 Starting video stream server on port {STREAM_PORT}...")
        stream_app.run(host='0.0.0.0', port=STREAM_PORT, threaded=True, debug=False)
    
    stream_thread = threading.Thread(target=run_stream_server, daemon=True)
    stream_thread.start()
    print(f"✅ Video stream available at http://0.0.0.0:{STREAM_PORT}/video_feed")
else:
    print("⚠️ Video streaming disabled")

# --- MAIN LOOP (Single Threaded MAVLink) ---
last_telem_send = time.time()

try:
    while True:
        # Read ALL messages (Non-blocking)
        msg = drone.recv_match(blocking=False)
        if msg:
            mtype = msg.get_type()
            
            if mtype == 'HEARTBEAT':
                LAST_HEARTBEAT = time.time()
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                DRONE_DATA["status"] = "ARMED" if is_armed else "DISARMED"
                cid = msg.custom_mode
                DRONE_DATA["mode"] = COPTER_MODES.get(cid, str(cid))
                
            elif mtype == 'GLOBAL_POSITION_INT':
                DRONE_DATA["lat"] = msg.lat / 1e7
                DRONE_DATA["lon"] = msg.lon / 1e7
                DRONE_DATA["alt"] = msg.relative_alt / 1000.0
                
            elif mtype == 'SYS_STATUS':
                DRONE_DATA["bat"] = msg.battery_remaining
                # Battery voltage: received in millivolts, convert to volts
                DRONE_DATA["bat_voltage"] = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else 0
                # Current draw: received in 10*milliamps, convert to amps
                DRONE_DATA["bat_current"] = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
                
                # Calculate estimated time remaining (minutes)
                # Formula: (capacity * remaining%) / current_draw / 60
                if DRONE_DATA["bat_current"] > 0.1:  # Only calculate if drawing meaningful current
                    remaining_mah = BATTERY_CAPACITY_MAH * (DRONE_DATA["bat"] / 100.0)
                    time_hours = remaining_mah / (DRONE_DATA["bat_current"] * 1000)
                    DRONE_DATA["bat_time_min"] = round(time_hours * 60, 1)
                else:
                    DRONE_DATA["bat_time_min"] = 0  # No current draw, can't estimate
                
            elif mtype == 'VFR_HUD':
                DRONE_DATA["speed"] = round(msg.groundspeed, 1)
                DRONE_DATA["heading"] = msg.heading
                
            elif mtype == 'ATTITUDE':
                # msg.yaw is in radians (-pi to pi)
                # convert to degrees (0-360)
                deg = math.degrees(msg.yaw)
                if deg < 0: deg += 360
                DRONE_DATA["heading"] = deg
            
            elif mtype == 'GPS_RAW_INT':
                # GPS satellite count
                DRONE_DATA["gps_sats"] = msg.satellites_visible
                DRONE_DATA["gps_fix"] = msg.fix_type
                
            elif mtype == 'COMMAND_ACK':
                res_text = MAV_RESULT_NAMES.get(msg.result, str(msg.result))
                print(f"🔔 ACK Received: Cmd={msg.command} Res={msg.result} ({res_text})")
                if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    # Provide hints for common failures
                    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        print("❌ ARM/DISARM refused: check mode, landed state, or use force=true")
                    elif msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        print("❌ TAKEOFF refused: Ensure GUIDED mode, armed, and valid GPS")
                        print(f"   Current state: Mode={DRONE_DATA.get('mode')} Status={DRONE_DATA.get('status')} GPS=({DRONE_DATA.get('lat')}, {DRONE_DATA.get('lon')})")
                    else:
                        print("❌ COMMAND FAILED/REFUSED")
            
            elif mtype == 'STATUSTEXT':
                print(f"🤖 FC MSG: {msg.text}")
                
            elif mtype == 'MISSION_REQUEST':
                # Handle mission upload parts naturally in loop
                seq = msg.seq
                print(f"   FC Requesting WP {seq}")
                send_mission_item(seq)
            
            elif mtype == 'MISSION_ACK':
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    print("✅ Mission Upload Complete!")
                    MISSION_UPLOAD_IN_PROGRESS = False
                    
                    # Auto-start mission if requested
                    if AUTO_START_MISSION:
                        print("🚀 Auto-starting mission...")
                        time.sleep(0.5)
                        
                        # Ensure armed first
                        if DRONE_DATA.get('status') != 'ARMED':
                            print("   ⚠️ Arming for mission...")
                            send_arm()
                            time.sleep(2)
                        
                        # Switch to AUTO mode to start mission
                        print("   ⚠️ Switching to AUTO mode...")
                        drone.set_mode('AUTO')
                        AUTO_START_MISSION = False
                else:
                    print(f"❌ Mission Upload Failed: Error {msg.type}")
                    MISSION_UPLOAD_IN_PROGRESS = False
                    AUTO_START_MISSION = False

        # Periodic Telemetry Publish
        if time.time() - last_telem_send > 0.5:
            # Publish to BOTH
            try:
                payload = json.dumps(DRONE_DATA)
                client1.publish(TOPIC_TELEM, payload)
                client2.publish(TOPIC_TELEM, payload)
            except Exception as e:
                pass # Ignore publish errors generally

            if int(time.time()) % 2 == 0: 
                print(f"\r📡 Sats:{DRONE_DATA.get('gps_sats',0)} Fix:{DRONE_DATA.get('gps_fix',0)} Bat:{DRONE_DATA.get('bat',0)}% Mode:{DRONE_DATA.get('mode')}", end="", flush=True)
            last_telem_send = time.time()
            
        time.sleep(0.001) # Reduce CPU

except KeyboardInterrupt:
    print("🚨 STOPPING")
