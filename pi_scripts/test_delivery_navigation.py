#!/usr/bin/env python3
"""
Test script for delivery drone navigation - simulates distance calculation and package drop logic
"""
import math
import time

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

def test_navigation():
    print("🧪 Testing Delivery Navigation System...")
    
    # Test coordinates (example: same general area)
    base_lat = 17.4239  # Example: Hyderabad
    base_lon = 78.4738
    
    target_lat = 17.4249  # ~100m north
    target_lon = 78.4748  # ~100m east
    
    # Calculate distance
    distance = calculate_distance(base_lat, base_lon, target_lat, target_lon)
    
    print(f"\n📍 Base Location: ({base_lat:.6f}, {base_lon:.6f})")
    print(f"📍 Target Location: ({target_lat:.6f}, {target_lon:.6f})")
    print(f"📏 Distance: {distance:.2f} meters")
    
    # Test threshold logic
    DROP_DISTANCE_THRESHOLD = 2.0
    
    if distance <= DROP_DISTANCE_THRESHOLD:
        print(f"✅ WITHIN drop threshold ({DROP_DISTANCE_THRESHOLD}m) - Package would be dropped")
    else:
        print(f"⏳ NOT YET at drop threshold ({DROP_DISTANCE_THRESHOLD}m) - Continue navigation")
    
    # Simulate approach
    print("\n🚁 Simulating approach...")
    simulated_distances = [150, 100, 50, 20, 10, 5, 3, 2, 1.5, 1, 0.5]
    
    for dist in simulated_distances:
        time.sleep(0.2)
        if dist <= DROP_DISTANCE_THRESHOLD:
            print(f"🎯 {dist:.1f}m - TARGET REACHED! Dropping package...")
            print("📦 PACKAGE DROPPED")
            print("🏠 Returning to launch...")
            break
        else:
            print(f"📍 {dist:.1f}m - Approaching...")
    
    print("\n✅ Navigation test complete!")
    print("\nℹ️  The delivery drone will:")
    print(f"   1. Monitor distance to target continuously")
    print(f"   2. Drop package when within {DROP_DISTANCE_THRESHOLD}m")
    print(f"   3. Automatically return to launch (RTL mode)")

if __name__ == "__main__":
    try:
        test_navigation()
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
