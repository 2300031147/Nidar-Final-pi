#!/usr/bin/env python3
"""
Test script for human detection - loads a sample image and runs detection
"""
import cv2
import numpy as np
import sys

def test_detection():
    print("🧪 Testing Human Detection System...")
    
    # Initialize HOG detector
    print("📷 Initializing HOG detector...")
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    print("✅ HOG detector initialized")
    
    # Option 1: Test with camera
    print("\n🎥 Testing with camera...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ Camera not available - skipping camera test")
    else:
        print("✅ Camera opened successfully")
        ret, frame = cap.read()
        
        if ret:
            print(f"✅ Frame captured: {frame.shape}")
            
            # Run detection
            print("🔍 Running detection...")
            boxes, weights = hog.detectMultiScale(
                frame,
                winStride=(8, 8),
                padding=(4, 4),
                scale=1.05,
                hitThreshold=0
            )
            
            print(f"📊 Found {len(boxes)} potential detections")
            
            # Filter by confidence
            confidence_threshold = 0.6
            good_detections = [(box, weight) for box, weight in zip(boxes, weights) if weight > confidence_threshold]
            
            print(f"✅ {len(good_detections)} detections above confidence threshold ({confidence_threshold})")
            
            # Draw boxes
            for (x, y, w, h), weight in good_detections:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, f"{weight:.2f}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Save result
            cv2.imwrite("detection_test_result.jpg", frame)
            print("💾 Result saved to: detection_test_result.jpg")
        
        cap.release()
    
    print("\n✅ Detection test complete!")
    print("\nℹ️  To test detection on scout drone:")
    print("   1. Copy this script to Raspberry Pi")
    print("   2. Run: python3 test_detection.py")
    print("   3. Check detection_test_result.jpg for marked detections")

if __name__ == "__main__":
    try:
        test_detection()
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
