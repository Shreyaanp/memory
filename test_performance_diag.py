#!/usr/bin/env python3
"""
Performance diagnostic - measure each operation's time
"""

import cv2
import numpy as np
import pyrealsense2 as rs
from hobot_dnn import pyeasy_dnn as dnn
import time

def main():
    print("="*60)
    print("Performance Diagnostic Test")
    print("="*60)
    
    # Load BPU model
    print("\n1. Loading BPU model...")
    t0 = time.time()
    models = dnn.load("/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm")
    print(f"   ✅ Model loaded in {(time.time()-t0)*1000:.1f}ms")
    
    # Start camera
    print("\n2. Starting RealSense...")
    t0 = time.time()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    print(f"   ✅ Camera started in {(time.time()-t0)*1000:.1f}ms")
    
    # Warm up
    print("\n3. Warming up...")
    for i in range(10):
        frames = pipeline.wait_for_frames()
    
    # Load face detector
    print("\n4. Loading face detector...")
    t0 = time.time()
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    print(f"   ✅ Loaded in {(time.time()-t0)*1000:.1f}ms")
    
    # Performance test loop
    print("\n5. Running 10-frame performance test...")
    print("-" * 60)
    
    times = {
        'capture': [],
        'face_detect': [],
        'bpu_inference': [],
        'draw': [],
        'encode': []
    }
    
    for i in range(10):
        # Capture
        t0 = time.time()
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        frame = np.asanyarray(color_frame.get_data())
        times['capture'].append((time.time()-t0)*1000)
        
        # Face detection
        t0 = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(50, 50))
        times['face_detect'].append((time.time()-t0)*1000)
        
        if len(faces) > 0:
            face = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = face
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
            # BPU inference
            t0 = time.time()
            try:
                roi = dnn.roi((x, y, x+w, y+h))
                outputs = models[0].forward(frame, roi)
                landmarks_raw = outputs[0].buffer[0]
                times['bpu_inference'].append((time.time()-t0)*1000)
                
                # Draw
                t0 = time.time()
                for j in range(0, min(212, len(landmarks_raw)), 2):
                    lx_raw = landmarks_raw[j]
                    ly_raw = landmarks_raw[j+1]
                    if lx_raw <= 1.0 and ly_raw <= 1.0:
                        lx = int(lx_raw * w + x)
                        ly = int(ly_raw * h + y)
                    else:
                        lx = int(lx_raw + x)
                        ly = int(ly_raw + y)
                    cv2.circle(frame, (lx, ly), 1, (0, 0, 255), -1)
                times['draw'].append((time.time()-t0)*1000)
                
            except Exception as e:
                print(f"   Frame {i+1}: BPU failed - {e}")
                times['bpu_inference'].append(0)
                times['draw'].append(0)
        else:
            print(f"   Frame {i+1}: No face detected")
            times['bpu_inference'].append(0)
            times['draw'].append(0)
        
        # Encode
        t0 = time.time()
        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        times['encode'].append((time.time()-t0)*1000)
        
        print(f"   Frame {i+1}: Cap={times['capture'][-1]:.1f}ms, Face={times['face_detect'][-1]:.1f}ms, "
              f"BPU={times['bpu_inference'][-1]:.1f}ms, Draw={times['draw'][-1]:.1f}ms, Enc={times['encode'][-1]:.1f}ms")
    
    pipeline.stop()
    
    # Summary
    print("\n" + "="*60)
    print("PERFORMANCE SUMMARY (average per frame)")
    print("="*60)
    
    for key, values in times.items():
        valid = [v for v in values if v > 0]
        if valid:
            avg = sum(valid) / len(valid)
            print(f"  {key.upper():15s}: {avg:6.1f}ms (min={min(valid):.1f}, max={max(valid):.1f})")
    
    total_avg = sum([sum(v)/len(v) for v in times.values()])
    print(f"\n  {'TOTAL':15s}: {total_avg:6.1f}ms per frame")
    print(f"  {'THEORETICAL FPS':15s}: {1000/total_avg:.1f} fps")
    print("="*60)

if __name__ == "__main__":
    main()


