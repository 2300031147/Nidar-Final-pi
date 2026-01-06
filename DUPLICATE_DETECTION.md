# Enhanced Duplicate Detection - Summary

## Changes Made

### 1. GPS Lock Validation (CRITICAL)
Before marking any detection, the system now verifies:
- ✅ **6+ GPS satellites** (minimum for reliable 3D fix)
- ✅ **GPS Fix Type 3+** (3D position fix)
- ✅ **Non-zero coordinates** (lat/lon != 0)
- ✅ **Not at default location** (lat != 16.506)

**Result**: Detections are ONLY recorded when GPS lock is strong and reliable.

---

### 2. Enhanced Duplicate Prevention
Three strict rules to prevent detecting the same human twice:

#### Rule 1: Global Cooldown
- **Minimum 30 seconds** between ANY detections
- Blocks rapid consecutive detections
- **Log**: `⏱️ Duplicate blocked by cooldown (X.Xs < 30s)`

#### Rule 2: Distance Filter
- **Minimum 15 meters** between detection locations
- Compares against ALL previous detections
- Uses Haversine formula for accurate GPS distance
- **Log**: `📏 Duplicate blocked by distance (X.Xm < 15m) - Detection #N`

#### Rule 3: Exact Location Match
- Prevents detecting exact same GPS coordinate twice
- Precision: 0.00001 degrees (~1.1 meters)
- **Log**: `🎯 Duplicate blocked by exact location match - Detection #N`

---

### 3. Unique Detection IDs
- Each detection gets a unique incrementing ID
- Starts at 1, increments forever
- Helps track which detection triggered which duplicate block
- Included in detection metadata

---

### 4. Enhanced Logging
Every detection attempt now logs:
- **GPS lock status**: Satellites, fix type, coordinates
- **Duplicate blocks**: Reason and which previous detection caused it
- **Successful detections**: ID, GPS stats, coordinates, altitude

Examples:
```
✅ GPS LOCKED - 12 satellites, Fix Type 3 - Detections enabled
👤 Detected 1 person(s) in frame
✅ DETECTION #1 MARKED - GPS Lock: 12 sats, Fix Type 3
   Location: (17.424900, 78.474800) @ 15.0m altitude

👤 Detected 1 person(s) in frame
📏 Duplicate blocked by distance (8.5m < 15m) - Detection #1
⚠️ Detection REJECTED - Duplicate location or too soon
```

---

## Configuration

Edit in `scout_tailscale.py`:

```python
DETECTION_COOLDOWN = 30  # Seconds between detections (increase for stricter filtering)
DETECTION_MIN_DISTANCE = 15  # Meters between detection points (increase to spread out)
```

**Stricter filtering example**:
```python
DETECTION_COOLDOWN = 60  # 1 minute between ANY detections
DETECTION_MIN_DISTANCE = 30  # 30 meters minimum distance
```

---

## Testing

### Test 1: GPS Lock Requirement
```bash
python3 scout_tailscale.py
# Wait for GPS lock
# Expected: "✅ GPS LOCKED - X satellites" every 10 seconds
```

### Test 2: Duplicate Prevention
1. Run scout with camera
2. Walk in front of camera
3. Wait for detection #1
4. Immediately walk in front again
5. Expected: "⏱️ Duplicate blocked by cooldown"

### Test 3: Distance Filter
1. Get detection at location A
2. Move 10 meters
3. Walk in front of camera
4. Expected: "📏 Duplicate blocked by distance (10.0m < 15m)"

---

## Benefits

1. **No false duplicates**: Same human never detected twice
2. **GPS accuracy**: Only marks detections with strong satellite lock
3. **Better spacing**: Detections spread out geographically
4. **Troubleshooting**: Detailed logs show why detections fail
5. **Unique tracking**: Each detection has traceable ID

---

## Migration Notes

**Existing deployments**: No action needed. System is backward compatible.

**GPS requirements**: Ensure drone has clear sky view with 6+ satellites before expecting detections.

**Testing**: Use `test_detection.py` to verify camera/detection works independently of GPS.
