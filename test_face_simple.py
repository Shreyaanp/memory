#!/usr/bin/env python3
"""
Minimal test - just display one frame with landmarks
No threading, no Flask - just basic test
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from hobot_dnn import pyeasy_dnn as dnn

def main():
    print("="*60)
    print("Simple Face Landmarks Test - Single Frame")
    print("="*60)
    
    # Load BPU model
    print("\n1. Loading BPU face landmark model...")
    model = dnn.load("/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm")
    print(f"   ✅ Model loaded: {model}")
    
    # Start camera
    print("\n2. Starting RealSense camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print("   ✅ Camera started")
    
    # Warm up camera
    print("\n3. Warming up camera (capturing 10 frames)...")
    for i in range(10):
        frames = pipeline.wait_for_frames()
    print("   ✅ Camera warmed up")
    
    # Capture one frame
    print("\n4. Capturing test frame...")
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    frame = np.asanyarray(color_frame.get_data())
    print(f"   ✅ Frame captured: {frame.shape}")
    
    # Detect face using OpenCV
    print("\n5. Detecting face...")
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    
    if len(faces) == 0:
        print("   ❌ No face detected!")
        print("   Please make sure your face is visible to the camera and try again.")
        pipeline.stop()
        return
    
    print(f"   ✅ Face detected: {len(faces)} face(s)")
    x, y, w, h = faces[0]
    print(f"      Position: x={x}, y={y}, w={w}, h={h}")
    
    # Draw face box
    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
    
    # Crop and prepare face for BPU
    print("\n6. Running BPU landmark detection...")
    padding = 20
    x1 = max(0, x - padding)
    y1 = max(0, y - padding)
    x2 = min(frame.shape[1], x + w + padding)
    y2 = min(frame.shape[0], y + h + padding)
    
    face_crop = frame[y1:y2, x1:x2]
    face_resized = cv2.resize(face_crop, (112, 112))
    
    # Run BPU inference
    try:
        inputs = [face_resized]
        outputs = model[0].forward(inputs)
        
        # Parse landmarks
        landmarks_raw = outputs[0].buffer[0]
        print(f"   ✅ BPU inference complete: {len(landmarks_raw)} values")
        
        # Draw landmarks
        landmarks = []
        for i in range(0, min(212, len(landmarks_raw)), 2):
            # Scale back to original coordinates
            lx = landmarks_raw[i] * (x2 - x1) / 112.0 + x1
            ly = landmarks_raw[i+1] * (y2 - y1) / 112.0 + y1
            landmarks.append((int(lx), int(ly)))
            
            # Draw point
            if i == 60:  # Nose tip (trying different index)
                cv2.circle(frame, (int(lx), int(ly)), 6, (0, 255, 0), -1)  # Green
            else:
                cv2.circle(frame, (int(lx), int(ly)), 2, (0, 0, 255), -1)  # Red
        
        print(f"   ✅ Drew {len(landmarks)} landmarks")
        
        # Find and highlight nose (approximate center point)
        if len(landmarks) > 50:
            nose_idx = 30  # Common nose tip index
            nx, ny = landmarks[nose_idx]
            cv2.circle(frame, (nx, ny), 8, (0, 255, 0), -1)
            cv2.line(frame, (nx-15, ny), (nx+15, ny), (0, 255, 0), 3)
            cv2.line(frame, (nx, ny-15), (nx, ny+15), (0, 255, 0), 3)
            print(f"   🎯 Nose position: ({nx}, {ny})")
        
    except Exception as e:
        print(f"   ❌ BPU inference failed: {e}")
        import traceback
        traceback.print_exc()
    
    # Save result
    print("\n7. Saving result...")
    output_path = "/home/mercleDev/test_landmarks_result.jpg"
    cv2.imwrite(output_path, frame)
    print(f"   ✅ Saved to: {output_path}")
    
    # Cleanup
    pipeline.stop()
    
    print("\n" + "="*60)
    print("✅ Test complete!")
    print(f"View the result: {output_path}")
    print("="*60)

if __name__ == "__main__":
    main()


