from flask import Flask, request, jsonify
from flask_cors import CORS
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
import json
import os
import time  # FIX: Added time import for auto_dispatch_delivery
# Removed eventlet to avoid conflicts with Paho MQTT threads

app = Flask(__name__)
CORS(app)
# Force threading mode for stability
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

BROKER = os.getenv('MQTT_BROKER', 'mqtt')
TOPIC_MISSION = "nidar/scout/mission"
TOPIC_DELIVERY_MISSION = "nidar/delivery/mission"
TOPIC_TELEM_SCOUT = "nidar/scout/telemetry"
TOPIC_TELEM_DELIVERY = "nidar/delivery/telemetry"
TOPIC_SURVIVOR = "nidar/delivery/target"
TOPIC_DETECTIONS = "nidar/scout/detections"

# Store detections in memory
detections_list = []

# AUTO-DISPATCH CONFIGURATION
AUTO_DISPATCH_ENABLED = True  # Set to False to require manual dispatch
DISPATCH_COOLDOWN = 60  # Minimum seconds between auto-dispatches
last_auto_dispatch_time = 0

def on_connect(client, userdata, flags, rc, properties=None):
    print("✅ API Connected to Broker")
    client.subscribe(TOPIC_TELEM_SCOUT)
    client.subscribe(TOPIC_TELEM_DELIVERY)
    client.subscribe(TOPIC_DETECTIONS)
    print(f"📡 Subscribed to telemetry and detection topics")

def on_message(client, userdata, msg):
    global detections_list
    try:
        payload_str = msg.payload.decode()
        print(f"📩 MSG Received on {msg.topic}") # Uncomment for verbose spam
        
        data = json.loads(payload_str)
        drone_type = "scout" if "scout" in msg.topic else "delivery"
        
        # Handle telemetry
        if "telemetry" in msg.topic:
            # print(f"   -> Forwarding Telemetry for {drone_type}")
            socketio.emit('telemetry_update', {"drone": drone_type, "data": data})
        
        # Handle detections
        elif "detections" in msg.topic:
            print(f"🚨 Detection received: {data}")
            detections_list.append(data)
            socketio.emit('detection_update', data)
            
            # AUTO-DISPATCH: Automatically send delivery drone
            if AUTO_DISPATCH_ENABLED:
                auto_dispatch_delivery(data.get('lat'), data.get('lon'))
            else:
                print("ℹ️  Auto-dispatch disabled - waiting for manual dispatch")
            
    except Exception as e:
        print(f"❌ Error processing message: {e}")

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
try:
    print(f"🔌 Connecting to Broker: {BROKER}")
    mqtt_client.connect(BROKER, 1883, 60)
    mqtt_client.loop_start()
except Exception as e:
    print(f"❌ MQTT Connection Fail: {e}")

@app.route('/upload_mission', methods=['POST'])
def upload_mission():
    data = request.json
    waypoints = data.get('waypoints')
    alt = data.get('alt', 20)
    speed = data.get('speed', 5)
    target_drone = data.get('drone', 'scout') # Default to scout
    
    topic = TOPIC_MISSION if target_drone == 'scout' else TOPIC_DELIVERY_MISSION
    
    payload = json.dumps({
        "type": "MISSION_UPLOAD", 
        "points": waypoints,
        "alt": alt,
        "speed": speed
    })
    mqtt_client.publish(topic, payload)
    return jsonify({"status": "Uploaded", "settings": {"alt": alt, "speed": speed}, "drone": target_drone})

# --- RESTORED COMMAND ROUTES ---
@app.route('/arm', methods=['POST'])
def arm_drone():
    mqtt_client.publish(TOPIC_MISSION, json.dumps({"type": "COMMAND", "act": "ARM"}))
    return jsonify({"status": "ARM Command Sent"})

@app.route('/disarm', methods=['POST'])
def disarm_drone():
    mqtt_client.publish(TOPIC_MISSION, json.dumps({"type": "COMMAND", "act": "DISARM"}))
    return jsonify({"status": "DISARM Command Sent"})

@app.route('/takeoff', methods=['POST'])
def takeoff_drone():
    data = request.json
    alt = data.get('alt', 10)  # Default to 10m if not specified
    mqtt_client.publish(TOPIC_MISSION, json.dumps({"type": "COMMAND", "act": "TAKEOFF", "alt": alt}))
    return jsonify({"status": f"TAKEOFF Command Sent (alt={alt}m)"})

@app.route('/land', methods=['POST'])
def land_drone():
    mqtt_client.publish(TOPIC_MISSION, json.dumps({"type": "COMMAND", "act": "LAND"}))
    return jsonify({"status": "LAND Command Sent"})

@app.route('/indoor_mode', methods=['POST'])
def indoor_mode():
    mqtt_client.publish(TOPIC_MISSION, json.dumps({"type": "COMMAND", "act": "INDOOR_MODE"}))
    return jsonify({"status": "INDOOR MODE SET"})

@app.route('/set_mode', methods=['POST'])
def set_flight_mode():
    data = request.json
    mode = data.get('mode')
    mqtt_client.publish(TOPIC_MISSION, json.dumps({"type": "COMMAND", "act": "SET_MODE", "mode": mode}))
    return jsonify({"status": f"Mode {mode} Sent"})

@app.route('/rescue', methods=['POST'])
def rescue_command():
    data = request.json
    payload = {
        "type": "COMMAND", 
        "act": "RESCUE", 
        "lat": data.get('lat'), 
        "lon": data.get('lon')
    }
    # Publish to Delivery Drone Topic
    mqtt_client.publish(TOPIC_SURVIVOR, json.dumps(payload))
    return jsonify({"status": "RESCUE LAUNCHED"})

@app.route('/rc_override', methods=['POST'])
def rc_override():
    data = request.json
    channels = data.get('channels', []) # List of 8 PWM values
    payload = {
        "type": "COMMAND",
        "act": "RC_OVERRIDE",
        "channels": channels
    }
    mqtt_client.publish(TOPIC_MISSION, json.dumps(payload))
    return jsonify({"status": "RC OVERRIDE SENT"})

@app.route('/detections', methods=['GET'])
def get_detections():
    """Return all detected humans."""
    return jsonify({"detections": detections_list, "count": len(detections_list)})

@app.route('/clear_detections', methods=['POST'])
def clear_detections():
    """Clear all stored detections."""
    global detections_list
    detections_list = []
    return jsonify({"status": "Detections cleared"})

@app.route('/dispatch_delivery', methods=['POST'])
def dispatch_delivery():
    """Dispatch delivery drone to a detected human location."""
    data = request.json
    lat = data.get('lat')
    lon = data.get('lon')
    
    if lat is None or lon is None:
        return jsonify({"error": "Missing lat/lon"}), 400
    
    payload = {
        "type": "DELIVERY",
        "lat": lat,
        "lon": lon
    }
    
    # Publish to delivery drone topic
    mqtt_client.publish(TOPIC_SURVIVOR, json.dumps(payload))
    print(f"🚁 Delivery dispatched to ({lat}, {lon})")
    
    return jsonify({"status": "Delivery dispatched", "target": {"lat": lat, "lon": lon}})

def auto_dispatch_delivery(lat, lon):
    """Automatically dispatch delivery drone when human detected (autonomous mode)."""
    global last_auto_dispatch_time
    
    if lat is None or lon is None:
        print("⚠️ Auto-dispatch skipped - Invalid coordinates")
        return
    
    # Cooldown check - prevent rapid consecutive dispatches
    time_since_last = time.time() - last_auto_dispatch_time
    if time_since_last < DISPATCH_COOLDOWN:
        print(f"⏱️ Auto-dispatch blocked by cooldown ({time_since_last:.1f}s < {DISPATCH_COOLDOWN}s)")
        return
    
    # Dispatch delivery
    payload = {
        "type": "DELIVERY",
        "lat": lat,
        "lon": lon
    }
    
    mqtt_client.publish(TOPIC_SURVIVOR, json.dumps(payload))
    last_auto_dispatch_time = time.time()
    
    print(f"🚁 AUTO-DISPATCH: Delivery sent to ({lat:.6f}, {lon:.6f})")
    socketio.emit('auto_dispatch_alert', {"lat": lat, "lon": lon})

if __name__ == '__main__':
    # allow_unsafe_werkzeug=True is required when using threading mode in Docker
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
