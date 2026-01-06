from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json
import time
import threading
import math

# 🔴 CONFIGURATION - YOUR TAILSCALE IP
BROKER_IP = "100.125.45.22"
TOPIC_MISSION = "nidar/delivery/mission"  # FIXED: Was scout/mission
TOPIC_TELEM = "nidar/delivery/telemetry"  # FIXED: Was scout/telemetry  
TOPIC_SURVIVOR = "nidar/delivery/target"

# PACKAGE DROP CONFIGURATION
DROP_SERVO_CHANNEL = 9  # AUX1 = Channel 9 (adjust as needed)
DROP_SERVO_PWM_OPEN = 1900  # PWM value to release package
DROP_SERVO_PWM_CLOSE = 1100  # PWM value to hold package
DROP_ALTITUDE = 3  # Altitude in meters to drop package
DROP_DISTANCE_THRESHOLD = 1.0  # Horizontal distance in meters to trigger drop

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

print("✅ Delivery Online")

# Request telemetry streams from the flight controller
print("📡 Requesting telemetry streams...")
drone.mav.request_data_stream_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    4, 1)  # 4Hz rate, enable=1

# --- STATE ---
DRONE_DATA = {
    "lat": 0, "lon": 0, "alt": 0, 
    "bat": 0,           # Battery percentage (0-100)
    "bat_voltage": 0,   # Voltage in volts
    "bat_current": 0,   # Current draw in amps  
    "bat_time_min": 0,  # Estimated minutes remaining
    "gps_sats": 0,      # Number of GPS satellites
    "status": "DISARMED", "speed": 0, "mode": "UNKNOWN"
}
BATTERY_CAPACITY_MAH = 5000  # Default battery capacity, adjust for your battery
LAST_HEARTBEAT = 0  # FIXED: Was missing
MISSION_UPLOAD_IN_PROGRESS = False  # Track mission upload state
AUTO_START_MISSION = False  # Flag to auto-start mission after upload
MISSION_POINTS = []  # FIXED: Was missing

# DELIVERY STATE
DELIVERY_ACTIVE = False
TARGET_LAT = 0
TARGET_LON = 0
PACKAGE_DROPPED = False
DELIVERY_ID_COUNTER = 0  # Track delivery missions
CURRENT_DELIVERY_ID = 0
RTL_INITIATED = False  # Track if RTL was initiated by delivery system
AUTO_DISARM_ENABLED = True  # Auto-disarm after landing
# --- COMMAND HELPERS (Send Only) ---
def send_arm():
    print("⚠️ Sending ARM Command...")
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    # Default to STABILIZE for safety unless mission running
    drone.set_mode('STABILIZE') 

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
    # Disable Auto Disarm Delay (DISARM_DELAY) - keeps it armed on bench
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
    print(f"📤 Uploading {len(waypoints)} points (Alt: {alt}m)...")
    
    # Store points with altitude
    mission_items = []
    
    if speed and speed > 0:
        mission_items.append({
            'type': 'SPEED',
            'speed': speed
        })

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
    """Check if delivery drone has valid GPS lock for safe navigation."""
    gps_sats = DRONE_DATA.get('gps_sats', 0)
    gps_fix = DRONE_DATA.get('gps_fix', 0)
    lat = DRONE_DATA.get('lat', 0)
    lon = DRONE_DATA.get('lon', 0)
    
    # Require stricter GPS for delivery missions:
    # - At least 8 satellites (stricter than scout for precision navigation)
    # - GPS fix type 3 or higher (3D fix)
    # - Non-zero coordinates
    # - Coordinates not at default location
    
    has_lock = (
        gps_sats >= 8 and
        gps_fix >= 3 and
        lat != 0 and lon != 0 and
        lat != 16.506  # Not default location
    )
    
    if not has_lock:
        print(f"⚠️ GPS Lock Check Failed: Sats={gps_sats}, Fix={gps_fix}, Pos=({lat:.6f}, {lon:.6f})")
        print(f"   ℹ️  Delivery requires 8+ satellites for precision navigation")
    
    return has_lock

def validate_target_coordinates(lat, lon):
    """Validate target coordinates are reasonable."""
    # Basic sanity checks
    if lat == 0 or lon == 0:
        print(f"❌ Invalid target: Zero coordinates ({lat}, {lon})")
        return False
    
    if abs(lat) > 90 or abs(lon) > 180:
        print(f"❌ Invalid target: Out of range ({lat}, {lon})")
        return False
    
    # Check if target is too far from current position
    current_lat = DRONE_DATA.get('lat', 0)
    current_lon = DRONE_DATA.get('lon', 0)
    
    if current_lat != 0 and current_lon != 0:
        distance = calculate_distance(current_lat, current_lon, lat, lon)
        if distance > 5000:  # 5km max range
            print(f"⚠️ Warning: Target is {distance:.0f}m away (>5km). Proceed with caution.")
    
    return True

def drop_package():
    """Trigger servo to release package - ONLY ONCE per mission."""
    global PACKAGE_DROPPED
    
    # CRITICAL: Prevent multiple drops in same mission
    if PACKAGE_DROPPED:
        print("⚠️ PACKAGE ALREADY DROPPED - Ignoring duplicate drop request")
        return False
    
    try:
        print(f"📦 DROPPING PACKAGE (Servo Channel {DROP_SERVO_CHANNEL})...")
        
        # Send servo command to release package
        drone.mav.command_long_send(
            drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # confirmation
            DROP_SERVO_CHANNEL,  # servo channel
            DROP_SERVO_PWM_OPEN,  # PWM value to open
            0, 0, 0, 0, 0  # unused params
        )
        
        # Mark as dropped IMMEDIATELY to prevent race conditions
        PACKAGE_DROPPED = True
        print("✅ Package dropped! (Drop flag set - no more drops this mission)")
        
        # Wait and close servo
        time.sleep(2)
        drone.mav.command_long_send(
            drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            DROP_SERVO_CHANNEL,
            DROP_SERVO_PWM_CLOSE,  # PWM value to close
            0, 0, 0, 0, 0
        )
        
        return True
        
    except Exception as e:
        print(f"❌ Package drop error: {e}")
        # Don't reset PACKAGE_DROPPED on error - still counts as attempt
        return False

def monitor_delivery():
    """Monitor distance to target and trigger package drop when close enough."""
    global DELIVERY_ACTIVE, PACKAGE_DROPPED, RTL_INITIATED  # FIX: Added RTL_INITIATED to globals
    
    print(f"📍 Monitoring delivery mission #{CURRENT_DELIVERY_ID}...")
    
    # Track stats
    min_distance_achieved = float('inf')
    gps_lock_warnings = 0
    
    while DELIVERY_ACTIVE and not PACKAGE_DROPPED:
        try:
            # Verify GPS lock during delivery
            if not has_gps_lock():
                gps_lock_warnings += 1
                if gps_lock_warnings > 5:
                    print("❌ GPS lock lost during delivery - ABORTING MISSION")
                    print("🏠 Returning to launch...")
                    drone.set_mode('RTL')
                    DELIVERY_ACTIVE = False
                    break
                time.sleep(1)
                continue
            else:
                gps_lock_warnings = 0  # Reset on good GPS
            
            current_lat = DRONE_DATA.get('lat', 0)
            current_lon = DRONE_DATA.get('lon', 0)
            current_alt = DRONE_DATA.get('alt', 0)
            
            if current_lat == 0 or current_lon == 0:
                time.sleep(0.5)
                continue
            
            # Calculate horizontal distance to target
            distance = calculate_distance(current_lat, current_lon, TARGET_LAT, TARGET_LON)
            
            # Track minimum distance
            if distance < min_distance_achieved:
                min_distance_achieved = distance
            
            # Check if we're close enough and at drop altitude
            if distance <= DROP_DISTANCE_THRESHOLD:
                print(f"🎯 Target reached! Distance: {distance:.1f}m (Min: {min_distance_achieved:.1f}m)")
                
                # Drop package immediately (No descent)
                drop_package()
                
                # Wait a moment
                time.sleep(2)
                
                # Return to launch
                print("🏠 Returning to launch...")
                drone.set_mode('RTL')
                global RTL_INITIATED
                RTL_INITIATED = True  # Mark RTL as initiated by delivery system
                
                DELIVERY_ACTIVE = False
                break
            else:
                # Print progress every 2 seconds
                if int(time.time()) % 2 == 0:
                    gps_sats = DRONE_DATA.get('gps_sats', 0)
                    print(f"\r📍 Distance: {distance:.1f}m | GPS: {gps_sats} sats | Alt: {current_alt:.1f}m", end="", flush=True)
            
            time.sleep(0.5)
            
        except Exception as e:
            print(f"❌ Delivery monitoring error: {e}")
            time.sleep(1)
    
    if PACKAGE_DROPPED:
        print(f"✅ Delivery mission #{CURRENT_DELIVERY_ID} complete! Min distance: {min_distance_achieved:.1f}m")
    else:
        print(f"⚠️ Delivery mission #{CURRENT_DELIVERY_ID} aborted.")

def execute_rescue(lat, lon):
    """Execute rescue mission - fly to survivor location and drop kit."""
    global DELIVERY_ACTIVE, TARGET_LAT, TARGET_LON, PACKAGE_DROPPED, DELIVERY_ID_COUNTER, CURRENT_DELIVERY_ID
    
    # Increment delivery counter
    DELIVERY_ID_COUNTER += 1
    CURRENT_DELIVERY_ID = DELIVERY_ID_COUNTER
    
    print(f"\n{'='*60}")
    print(f"🚑 DELIVERY MISSION #{CURRENT_DELIVERY_ID} INITIATED")
    print(f"{'='*60}")
    print(f"📍 Target: ({lat:.6f}, {lon:.6f})")
    
    # CRITICAL: Validate GPS lock before starting mission
    if not has_gps_lock():
        print("❌ MISSION ABORTED - No GPS lock (requires 8+ satellites, 3D fix)")
        print("   ⚠️  Cannot navigate safely without GPS. Wait for satellite lock.")
        return False
    
    # Validate target coordinates
    if not validate_target_coordinates(lat, lon):
        print("❌ MISSION ABORTED - Invalid target coordinates")
        return False
    
    # Calculate distance to target
    current_lat = DRONE_DATA.get('lat', 0)
    current_lon = DRONE_DATA.get('lon', 0)
    if current_lat != 0 and current_lon != 0:
        distance_to_target = calculate_distance(current_lat, current_lon, lat, lon)
        print(f"📏 Distance to target: {distance_to_target:.1f}m")
    
    # Display GPS status
    gps_sats = DRONE_DATA.get('gps_sats', 0)
    gps_fix = DRONE_DATA.get('gps_fix', 0)
    print(f"🛰️  GPS Status: {gps_sats} satellites, Fix Type {gps_fix}")
    print(f"📍 Current Position: ({current_lat:.6f}, {current_lon:.6f})")
    
    # CRITICAL: Reset delivery state for NEW mission
    DELIVERY_ACTIVE = True
    TARGET_LAT = lat
    TARGET_LON = lon
    PACKAGE_DROPPED = False  # Reset drop flag for new mission
    
    print(f"✅ Package drop flag reset for mission #{CURRENT_DELIVERY_ID}")
    
    try:
        # Step 1: Ensure armed
        if DRONE_DATA.get('status') != 'ARMED':
            print("⚠️ Arming for rescue mission...")
            send_arm()
            time.sleep(2)
        
        # Verify GPS still good after arming
        if not has_gps_lock():
            print("❌ GPS lock lost after arming - ABORTING")
            DELIVERY_ACTIVE = False
            return False
        
        # Step 2: Takeoff if not airborne
        if DRONE_DATA.get('alt', 0) < 2:  # If altitude less than 2m, takeoff
            print("🛫 Taking off...")
            threading.Thread(target=handle_takeoff_command, args=(10,), daemon=True).start()
            time.sleep(5)  # Wait for takeoff
        
        # Verify GPS after takeoff
        if not has_gps_lock():
            print("❌ GPS lock lost during takeoff - ABORTING")
            drone.set_mode('LAND')
            DELIVERY_ACTIVE = False
            return False
        
        # Step 3: Switch to GUIDED mode
        if DRONE_DATA.get('mode') != 'GUIDED':
            print("⚠️ Switching to GUIDED mode...")
            drone.set_mode('GUIDED')
            time.sleep(1)
        
        # Step 4: Send waypoint command to fly to survivor (with precision offset)
        # Offset by 1.5m (approximate, depends on GPS accuracy)
        print(f"🛩️ Flying to survivor at {lat:.6f}, {lon:.6f}...")
        drone.mav.mission_item_int_send(
            drone.target_system, drone.target_component,
            0,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 1,  # current=2 (GUIDED mode target), autocontinue=1
            0, 0, 0, 0,  # params
            int(lat * 1e7),
            int(lon * 1e7),
            10  # Altitude 10m
        )
        
        # Step 5: Start monitoring thread
        monitor_thread = threading.Thread(target=monitor_delivery, daemon=True)
        monitor_thread.start()
        
        print("✅ Delivery mission started! Monitoring progress...")
        print(f"{'='*60}\n")
        return True
        
    except Exception as e:
        print(f"❌ RESCUE Error: {e}")
        DELIVERY_ACTIVE = False
        return False

# --- MQTT CALLBACKS ---
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        if 'type' in data and data['type'] == "COMMAND":
            act = data.get('act')
            if act == "ARM": send_arm()
            elif act == "DISARM": send_disarm()
            elif act == "LAND": 
                drone.set_mode('LAND')
                print("🛬 LANDING")
            elif act == "TAKEOFF":
                # Use enhanced TAKEOFF with proper alt parameter
                alt = float(data.get('alt', 10))
                threading.Thread(target=handle_takeoff_command, args=(alt,), daemon=True).start()
            elif act == "INDOOR_MODE": set_indoor_mode()
            elif act == "RC_OVERRIDE": override_rc(data.get('channels', []))
            elif act == "SET_MODE":
                mode = data.get('mode')
                print(f"⚠️ Mode -> {mode}")
                drone.set_mode(mode)
        
        elif 'lat' in data and 'lon' in data:
            execute_rescue(data['lat'], data['lon'])
        elif data.get('type') == "MISSION_UPLOAD":
             waypoints = data.get('points')
             alt = data.get('alt', 20)
             speed = data.get('speed', 5)
             upload_mission_to_fc(waypoints, alt=alt, speed=speed)

    except Exception as e:
        print(f"Msg Error: {e}")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("✅ Connected to GCS MQTT Broker")
        client.subscribe(TOPIC_MISSION)
        client.subscribe(TOPIC_SURVIVOR)
        print(f"📡 Subscribed to {TOPIC_MISSION} & {TOPIC_SURVIVOR}")
    else:
        print(f"❌ Connection Failed: Code {rc}")

client.on_connect = on_connect
client.on_message = on_message

while True:
    try:
        client.connect(BROKER_IP, 1883, 60)
        break
    except Exception as e:
        print(f"⚠️ MQTT Connection Failed: {e}")
        print("   Retrying in 5 seconds...")
        time.sleep(5)

client.loop_start()

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
                current_mode = COPTER_MODES.get(cid, str(cid))
                DRONE_DATA["mode"] = current_mode
                
                # AUTO-DISARM: After RTL completes and drone lands
                if RTL_INITIATED and AUTO_DISARM_ENABLED:
                    # Check if in LAND mode (RTL transitions to LAND near home)
                    if current_mode == 'LAND' and is_armed:
                        # Check altitude - if very low, we've landed
                        alt = DRONE_DATA.get('alt', 999)
                        if alt < 0.5:  # Less than 0.5m altitude = landed
                            print(f"✅ Landed (alt={alt:.2f}m) - Auto-disarming in 3 seconds...")
                            time.sleep(3)  # Wait to ensure stable landing
                            send_disarm(force=True)
                            print("✅ Auto-disarm complete - Mission cycle finished")
                            RTL_INITIATED = False  # Reset flag
                
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
                if DRONE_DATA["bat_current"] > 0.1:  # Only calculate if drawing meaningful current
                    remaining_mah = BATTERY_CAPACITY_MAH * (DRONE_DATA["bat"] / 100.0)
                    time_hours = remaining_mah / (DRONE_DATA["bat_current"] * 1000)
                    DRONE_DATA["bat_time_min"] = round(time_hours * 60, 1)
                else:
                    DRONE_DATA["bat_time_min"] = 0
                
            elif mtype == 'VFR_HUD':
                DRONE_DATA["speed"] = round(msg.groundspeed, 1)
            
            elif mtype == 'GPS_RAW_INT':
                # GPS satellite count
                DRONE_DATA["gps_sats"] = msg.satellites_visible
                DRONE_DATA["gps_fix"] = msg.fix_type
                
            elif mtype == 'COMMAND_ACK':
                print(f"🔔 ACK Received: Cmd={msg.command} Res={msg.result}")
                if msg.result != 0:
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
            client.publish(TOPIC_TELEM, json.dumps(DRONE_DATA))
            # print(f"📡 Telem: {DRONE_DATA['status']} Alt:{DRONE_DATA['alt']}")
            if int(time.time()) % 2 == 0: 
                 print(f"\r📡 Sats:{DRONE_DATA.get('gps_sats',0)} Fix:{DRONE_DATA.get('gps_fix',0)} Bat:{DRONE_DATA.get('bat',0)}% Mode:{DRONE_DATA.get('mode')}", end="", flush=True) 
            last_telem_send = time.time()
            
        time.sleep(0.001) # Reduce CPU

except KeyboardInterrupt:
    print("🚨 STOPPING")