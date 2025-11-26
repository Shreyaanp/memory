#!/usr/bin/env python3
"""
Test face_landmark.tflite model CORRECTLY:
1. Detect face using OpenCV
2. Crop face ROI
3. Run face_landmark.tflite on cropped face
4. Map landmarks back to original image
"""

import numpy as np
import cv2
import sys
import os

# Use tflite_runtime
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    import tensorflow as tf
    Interpreter = tf.lite.Interpreter

# Paths
MODEL_PATH = "/home/mercleDev/codebase/tools/face_landmark.tflite"
IMAGE_PATH = "/home/mercleDev/Pictures/test.jpg"
OUTPUT_PATH = "/home/mercleDev/Pictures/test_landmarks_result.jpg"

def detect_face_opencv(image):
    """Detect face using OpenCV Haar cascade"""
    # Load face detector
    cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
    face_cascade = cv2.CascadeClassifier(cascade_path)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 5, minSize=(50, 50))
    
    if len(faces) == 0:
        print("No face detected! Using full image...")
        h, w = image.shape[:2]
        return (0, 0, w, h)
    
    # Return largest face
    faces = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)
    return faces[0]  # (x, y, w, h)

def expand_roi(x, y, w, h, img_w, img_h, scale=1.5):
    """Expand ROI to include more context around face"""
    cx, cy = x + w // 2, y + h // 2
    new_w = int(w * scale)
    new_h = int(h * scale)
    
    new_x = max(0, cx - new_w // 2)
    new_y = max(0, cy - new_h // 2)
    new_w = min(new_w, img_w - new_x)
    new_h = min(new_h, img_h - new_y)
    
    return new_x, new_y, new_w, new_h

def main():
    print("=" * 60)
    print("Face Landmark Test - Using face_landmark.tflite")
    print("=" * 60)
    
    # Load model
    print(f"\n[1] Loading model: {MODEL_PATH}")
    interpreter = Interpreter(model_path=MODEL_PATH)
    interpreter.allocate_tensors()
    
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    input_shape = input_details[0]['shape']
    print(f"    Input shape: {input_shape}")  # [1, 192, 192, 3]
    
    # Load image
    print(f"\n[2] Loading image: {IMAGE_PATH}")
    image = cv2.imread(IMAGE_PATH)
    if image is None:
        print(f"ERROR: Could not load {IMAGE_PATH}")
        return 1
    
    img_h, img_w = image.shape[:2]
    print(f"    Image size: {img_w}x{img_h}")
    
    # Detect face
    print(f"\n[3] Detecting face...")
    fx, fy, fw, fh = detect_face_opencv(image)
    print(f"    Face detected at: x={fx}, y={fy}, w={fw}, h={fh}")
    
    # Expand ROI slightly
    rx, ry, rw, rh = expand_roi(fx, fy, fw, fh, img_w, img_h, scale=1.3)
    print(f"    Expanded ROI: x={rx}, y={ry}, w={rw}, h={rh}")
    
    # Crop face region
    print(f"\n[4] Cropping face ROI...")
    face_crop = image[ry:ry+rh, rx:rx+rw]
    
    # Preprocess for model (192x192, RGB, float32, normalized)
    face_rgb = cv2.cvtColor(face_crop, cv2.COLOR_BGR2RGB)
    face_resized = cv2.resize(face_rgb, (192, 192))
    face_normalized = face_resized.astype(np.float32) / 255.0
    face_input = np.expand_dims(face_normalized, axis=0)
    
    print(f"    Input tensor shape: {face_input.shape}")
    
    # Run inference
    print(f"\n[5] Running face_landmark.tflite inference...")
    interpreter.set_tensor(input_details[0]['index'], face_input)
    interpreter.invoke()
    
    # Get outputs
    landmarks_raw = interpreter.get_tensor(output_details[0]['index'])
    confidence = interpreter.get_tensor(output_details[1]['index'])
    
    print(f"    Landmarks shape: {landmarks_raw.shape}")
    print(f"    Confidence: {confidence.flatten()[0]:.4f}")
    
    # Parse landmarks (468 x 3)
    landmarks = landmarks_raw.flatten().reshape(-1, 3)
    print(f"    Number of landmarks: {len(landmarks)}")
    
    # Landmarks are in 192x192 space, convert to ROI space then to original image
    print(f"\n[6] Drawing landmarks on image...")
    result = image.copy()
    
    # Draw ROI rectangle
    cv2.rectangle(result, (rx, ry), (rx + rw, ry + rh), (255, 0, 0), 2)
    
    # Draw landmarks
    for i, (lx, ly, lz) in enumerate(landmarks):
        # Convert from 192x192 space to ROI space
        roi_x = (lx / 192.0) * rw
        roi_y = (ly / 192.0) * rh
        
        # Convert from ROI space to original image space
        img_x = int(rx + roi_x)
        img_y = int(ry + roi_y)
        
        # Draw point
        cv2.circle(result, (img_x, img_y), 1, (0, 255, 0), -1)
    
    # Draw key points larger
    key_indices = [1, 33, 133, 362, 263, 61, 291, 17, 0, 4, 5, 6]  # nose, eyes, mouth
    for idx in key_indices:
        if idx < len(landmarks):
            lx, ly, lz = landmarks[idx]
            img_x = int(rx + (lx / 192.0) * rw)
            img_y = int(ry + (ly / 192.0) * rh)
            cv2.circle(result, (img_x, img_y), 3, (0, 0, 255), -1)
    
    # Add text
    cv2.putText(result, "face_landmark.tflite - 468 points", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Save
    print(f"\n[7] Saving result to: {OUTPUT_PATH}")
    cv2.imwrite(OUTPUT_PATH, result)
    
    print("\n" + "=" * 60)
    print("DONE!")
    print("=" * 60)
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

