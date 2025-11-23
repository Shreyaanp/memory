#!/usr/bin/env python3
"""
Debug script - just capture and save what the camera sees
"""

import cv2
import numpy as np
import pyrealsense2 as rs

def main():
    print("="*60)
    print("Camera View Test - See what RealSense sees")
    print("="*60)
    
    # Start camera
    print("\nStarting RealSense camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print("✅ Camera started")
    
    # Warm up
    print("\nWarming up (10 frames)...")
    for i in range(10):
        frames = pipeline.wait_for_frames()
    
    # Capture multiple frames
    print("\nCapturing 5 test frames...")
    for i in range(5):
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())
        
        # Try face detection with different parameters
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Try multiple detection settings
        faces1 = face_cascade.detectMultiScale(gray, 1.1, 3, minSize=(30, 30))
        faces2 = face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(50, 50))
        
        # Draw all detected faces
        frame_annotated = frame.copy()
        
        # Draw with relaxed settings (green)
        for (x, y, w, h) in faces1:
            cv2.rectangle(frame_annotated, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame_annotated, "Relaxed", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw with strict settings (blue)
        for (x, y, w, h) in faces2:
            cv2.rectangle(frame_annotated, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame_annotated, "Strict", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Add info
        cv2.putText(frame_annotated, f"Frame {i+1}/5", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame_annotated, f"Relaxed: {len(faces1)} faces", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame_annotated, f"Strict: {len(faces2)} faces", (10, 85), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Save both raw and annotated
        cv2.imwrite(f"/home/mercleDev/camera_raw_{i+1}.jpg", frame)
        cv2.imwrite(f"/home/mercleDev/camera_annotated_{i+1}.jpg", frame_annotated)
        
        print(f"  Frame {i+1}: Relaxed={len(faces1)}, Strict={len(faces2)}")
    
    pipeline.stop()
    
    print("\n" + "="*60)
    print("✅ Test complete!")
    print("\nSaved files:")
    print("  - camera_raw_1.jpg to camera_raw_5.jpg (what camera sees)")
    print("  - camera_annotated_1.jpg to camera_annotated_5.jpg (with face boxes)")
    print("\nNext: Copy these images back to your laptop to see them")
    print("="*60)

if __name__ == "__main__":
    main()


