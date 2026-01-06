# Fully Autonomous Operation - Complete Guide

## Overview

The system now operates **100% autonomously** without any human interaction:

1. **Scout detects human** → GPS coordinates marked
2. **Backend auto-dispatches delivery** → No button click needed
3. **Delivery drone flies to target** → Autonomous navigation
4. **Package dropped** → Servo triggered at 2m proximity
5. **Return to launch (RTL)** → Automatic mode switch
6. **Auto-land** → RTL handles landing
7. **Auto-disarm** → Motors stop after touchdown

---

## Configuration

### Backend Auto-Dispatch

**File**: `backend/server.py`

```python
# Enable/disable auto-dispatch
AUTO_DISPATCH_ENABLED = True  # Set to False for manual dispatch

# Cooldown between auto-dispatches (prevents rapid consecutive missions)
DISPATCH_COOLDOWN = 60  # Seconds (default: 60)
```

**Behavior**:
- When `True`: Delivery dispatched immediately when human detected
- When `False`: Requires manual button click from website
- Cooldown prevents sending multiple deliveries for same person

---

### Delivery Drone Auto-Disarm

**File**: `pi_scripts/delivery_tailscale.py`

```python
# Enable/disable auto-disarm after landing
AUTO_DISARM_ENABLED = True  # Set to False to keep armed after landing
```

**Behavior**:
- Monitors altitude after RTL initiated
- When altitude < 0.5m (landed), waits 3 seconds
- Automatically disarms motors
- Mission cycle complete

---

## Full Autonomous Cycle

### Timeline

```
00:00 - Scout flying, camera scanning
00:15 - Human detected at (17.4249, 78.4748)
      ✅ GPS lock verified (12 sats)
      ✅ Duplicate check passed
      🚨 Detection #1 published to MQTT

00:16 - Backend receives detection
      ✅ Auto-dispatch enabled
      ✅ Cooldown check passed
      🚁 AUTO-DISPATCH: Delivery sent to (17.4249, 78.4748)

00:17 - Delivery drone receives coordinates
      ✅ GPS lock verified (10 sats, Fix Type 3)
      ✅ Target validated
      📏 Distance to target: 156.2m
      🚑 DELIVERY MISSION #1 INITIATED

00:18 - Arming and takeoff
      ⚠️ Arming for rescue mission...
      ✅ Armed
      🛫 Taking off to 10m...

00:25 - En route to target
      📍 Distance: 85.3m | GPS: 11 sats | Alt: 10.1m
      📍 Distance: 52.7m | GPS: 12 sats | Alt: 10.0m
      📍 Distance: 28.4m | GPS: 11 sats | Alt: 10.2m

00:35 - Target reached
      🎯 Target reached! Distance: 1.8m (Min: 1.5m)
      ⬇️ Descending to drop altitude (3m)...
      
00:38 - Package drop
      📦 DROPPING PACKAGE (Servo Channel 9)...
      ✅ Package dropped! (Drop flag set)
      🏠 Returning to launch...

00:40 - RTL mode activated
      Mode: RTL
      📍 Distance to home: 145.8m
      
00:50 - Approaching home
      📍 Distance to home: 45.2m
      Alt: 8.5m (descending)

00:55 - Landing initiated
      Mode: LAND
      Alt: 2.3m → 1.1m → 0.4m

00:57 - Touchdown and auto-disarm
      ✅ Landed (alt=0.3m) - Auto-disarming in 3 seconds...
      ✅ Auto-disarm complete - Mission cycle finished
      Status: DISARMED

MISSION COMPLETE - No human interaction required!
```

---

## Safety Features

### 1. GPS Lock Requirements
- **Scout**: 6+ satellites for detection
- **Delivery**: 8+ satellites for mission start
- **In-flight**: Continuous GPS monitoring, auto-abort on loss

### 2. Duplicate Prevention
- **Detection**: 30s cooldown + 15m distance filter
- **Dispatch**: 60s cooldown between auto-dispatches
- **Drop**: Single drop per mission

### 3. Auto-Abort Conditions
- GPS lock lost during flight → RTL
- Target coordinates invalid → Mission rejected
- Package already dropped → Duplicate attempts blocked

### 4. Auto-Disarm Safety
- Only after RTL-initiated landing
- Waits for altitude < 0.5m
- 3-second stabilization delay
- Forced disarm (bypasses safety checks)

---

## Disabling Autonomous Features

### Manual Dispatch Mode

**Edit**: `backend/server.py`
```python
AUTO_DISPATCH_ENABLED = False
```

**Result**: Detections appear on website, but delivery requires manual button click.

---

### Manual Disarm Mode

**Edit**: `pi_scripts/delivery_tailscale.py`
```python
AUTO_DISARM_ENABLED = False
```

**Result**: Drone lands but stays armed. Requires manual DISARM command.

---

## Testing Autonomous Operation

### Test 1: Full Cycle (No Intervention)
```bash
# 1. Start all components
docker-compose up -d  # Backend
python3 scout_tailscale.py  # Scout Pi
python3 delivery_tailscale.py  # Delivery Pi

# 2. Arm and launch scout drone
# 3. Walk in front of scout camera
# 4. Watch delivery drone execute mission automatically
# 5. Confirm auto-disarm after landing

# Expected: Complete cycle without any button clicks
```

---

### Test 2: Auto-Dispatch Cooldown
```bash
# 1. Get first detection
# 2. Immediately walk in front of camera again
# 3. Expected: 
#    - Detection #2 logged
#    - Auto-dispatch blocked by cooldown
#    - "⏱️ Auto-dispatch blocked by cooldown (5.2s < 60s)"
```

---

### Test 3: Auto-Disarm Verification
```bash
# 1. Complete delivery mission
# 2. Watch RTL sequence
# 3. Monitor altitude during landing
# 4. Expected:
#    - "✅ Landed (alt=0.3m) - Auto-disarming in 3 seconds..."
#    - Status changes to DISARMED
#    - Motors stop
```

---

## Monitoring & Logs

### Backend Logs
```
🚨 Detection received: {'lat': 17.4249, 'lon': 78.4748, ...}
🚁 AUTO-DISPATCH: Delivery sent to (17.424900, 78.474800)
```

### Scout Logs
```
👤 Detected 1 person(s) in frame
✅ DETECTION #1 MARKED - GPS Lock: 12 sats, Fix Type 3
```

### Delivery Logs
```
🚑 DELIVERY MISSION #1 INITIATED
📍 Distance: 1.8m | GPS: 11 sats | Alt: 10.0m
📦 DROPPING PACKAGE (Servo Channel 9)...
✅ Landed (alt=0.3m) - Auto-disarming in 3 seconds...
```

---

## Benefits

✅ **Zero human intervention** - Complete autonomous rescue operation  
✅ **Faster response** - No waiting for operator to click dispatch  
✅ **Safer operations** - Auto-disarm prevents accidents after landing  
✅ **Scalable** - Can handle multiple detections in sequence  
✅ **Auditable** - Complete logs from detection to disarm  

---

## System Requirements

- Scout drone with camera and GPS (6+ satellites)
- Delivery drone with servo and GPS (8+ satellites)  
- MQTT broker running
- Backend server running
- Clear line of sight for GPS lock
- Geofenced operating area (recommended)

---

## Emergency Override

**Manual control always available**:
- DISARM button works anytime
- LAND button for emergency landing
- RTL button for immediate return
- Mode changes via website

**To stop auto-dispatch**:
- Set `AUTO_DISPATCH_ENABLED = False`
- Restart backend
- Detections still logged, but no auto-dispatch
