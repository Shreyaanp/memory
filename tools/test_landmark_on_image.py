#!/usr/bin/env python3
"""
Test face_landmark.tflite model on an image and save result with landmarks drawn
"""

import numpy as np
import cv2
import sys
import os

# Try to import tflite_runtime first, fall back to tensorflow
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    import tensorflow as tf
    Interpreter = tf.lite.Interpreter

# MediaPipe Face Mesh landmark connections for visualization
# These define which landmarks should be connected with lines
FACE_OVAL = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136, 172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109, 10]

LIPS_OUTER = [61, 146, 91, 181, 84, 17, 314, 405, 321, 375, 291, 409, 270, 269, 267, 0, 37, 39, 40, 185, 61]
LIPS_INNER = [78, 95, 88, 178, 87, 14, 317, 402, 318, 324, 308, 415, 310, 311, 312, 13, 82, 81, 80, 191, 78]

LEFT_EYE = [263, 249, 390, 373, 374, 380, 381, 382, 362, 398, 384, 385, 386, 387, 388, 466, 263]
RIGHT_EYE = [33, 7, 163, 144, 145, 153, 154, 155, 133, 173, 157, 158, 159, 160, 161, 246, 33]

LEFT_EYEBROW = [276, 283, 282, 295, 285, 300, 293, 334, 296, 336]
RIGHT_EYEBROW = [46, 53, 52, 65, 55, 70, 63, 105, 66, 107]

NOSE = [168, 6, 197, 195, 5, 4, 1, 19, 94, 2]

def preprocess_image(image, target_size=(192, 192)):
    """Preprocess image for the model"""
    # Convert BGR to RGB
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Resize to model input size
    resized = cv2.resize(rgb, target_size)
    
    # Normalize to [0, 1]
    normalized = resized.astype(np.float32) / 255.0
    
    # Add batch dimension
    batched = np.expand_dims(normalized, axis=0)
    
    return batched

def draw_landmarks(image, landmarks, connections=None, color=(0, 255, 0), thickness=1, radius=1):
    """Draw landmarks on image"""
    h, w = image.shape[:2]
    
    # Draw points
    for i, (x, y, z) in enumerate(landmarks):
        # Scale to image size
        px = int(x * w)
        py = int(y * h)
        
        # Draw point
        cv2.circle(image, (px, py), radius, color, -1)
    
    # Draw connections if provided
    if connections:
        for connection in connections:
            for j in range(len(connection) - 1):
                idx1 = connection[j]
                idx2 = connection[j + 1]
                
                if idx1 < len(landmarks) and idx2 < len(landmarks):
                    x1 = int(landmarks[idx1][0] * w)
                    y1 = int(landmarks[idx1][1] * h)
                    x2 = int(landmarks[idx2][0] * w)
                    y2 = int(landmarks[idx2][1] * h)
                    
                    cv2.line(image, (x1, y1), (x2, y2), color, thickness)
    
    return image

def main():
    # Paths
    model_path = "/home/mercleDev/codebase/tools/face_landmark.tflite"
    image_path = "/home/mercleDev/Pictures/test.jpg"
    output_path = "/home/mercleDev/Pictures/test_landmarks_result.jpg"
    
    print("="*60)
    print("Face Landmark Detection Test")
    print("="*60)
    
    # Load model
    print(f"\nLoading model: {model_path}")
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()
    
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    print(f"Input shape: {input_details[0]['shape']}")
    print(f"Output shape: {output_details[0]['shape']}")
    
    # Load image
    print(f"\nLoading image: {image_path}")
    image = cv2.imread(image_path)
    if image is None:
        print(f"ERROR: Could not load image: {image_path}")
        return 1
    
    original_h, original_w = image.shape[:2]
    print(f"Image size: {original_w}x{original_h}")
    
    # Preprocess
    print("\nPreprocessing image...")
    input_data = preprocess_image(image)
    print(f"Input tensor shape: {input_data.shape}")
    
    # Run inference
    print("\nRunning inference...")
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    
    # Get output
    landmarks_output = interpreter.get_tensor(output_details[0]['index'])
    confidence_output = interpreter.get_tensor(output_details[1]['index'])
    
    print(f"Landmarks output shape: {landmarks_output.shape}")
    print(f"Confidence output: {confidence_output.flatten()[0]:.4f}")
    
    # Parse landmarks (468 points x 3 coordinates = 1404)
    landmarks_flat = landmarks_output.flatten()
    num_landmarks = len(landmarks_flat) // 3
    print(f"Number of landmarks: {num_landmarks}")
    
    # Reshape to (468, 3)
    landmarks = landmarks_flat.reshape(-1, 3)
    
    # The landmarks are in relative coordinates but may need adjustment
    # Check the range of values
    print(f"\nLandmarks stats:")
    print(f"  X range: {landmarks[:, 0].min():.4f} to {landmarks[:, 0].max():.4f}")
    print(f"  Y range: {landmarks[:, 1].min():.4f} to {landmarks[:, 1].max():.4f}")
    print(f"  Z range: {landmarks[:, 2].min():.4f} to {landmarks[:, 2].max():.4f}")
    
    # Normalize landmarks to [0, 1] range if they're not already
    # The face_landmark model outputs coordinates in 192x192 space
    landmarks_normalized = landmarks.copy()
    landmarks_normalized[:, 0] = landmarks[:, 0] / 192.0  # x
    landmarks_normalized[:, 1] = landmarks[:, 1] / 192.0  # y
    # z is depth, keep as is for now
    
    print(f"\nNormalized landmarks stats:")
    print(f"  X range: {landmarks_normalized[:, 0].min():.4f} to {landmarks_normalized[:, 0].max():.4f}")
    print(f"  Y range: {landmarks_normalized[:, 1].min():.4f} to {landmarks_normalized[:, 1].max():.4f}")
    
    # Draw landmarks on image
    print("\nDrawing landmarks...")
    result_image = image.copy()
    
    # Draw all points first (small green dots)
    for i, (x, y, z) in enumerate(landmarks_normalized):
        px = int(x * original_w)
        py = int(y * original_h)
        cv2.circle(result_image, (px, py), 1, (0, 255, 0), -1)
    
    # Draw connections for facial features
    connections = [
        (FACE_OVAL, (200, 180, 130)),      # Face oval - beige
        (LIPS_OUTER, (0, 0, 255)),          # Lips outer - red
        (LIPS_INNER, (0, 100, 255)),        # Lips inner - orange
        (LEFT_EYE, (255, 255, 0)),          # Left eye - cyan
        (RIGHT_EYE, (255, 255, 0)),         # Right eye - cyan
        (LEFT_EYEBROW, (255, 200, 0)),      # Left eyebrow - light cyan
        (RIGHT_EYEBROW, (255, 200, 0)),     # Right eyebrow - light cyan
    ]
    
    for connection, color in connections:
        for j in range(len(connection) - 1):
            idx1 = connection[j]
            idx2 = connection[j + 1]
            
            if idx1 < len(landmarks_normalized) and idx2 < len(landmarks_normalized):
                x1 = int(landmarks_normalized[idx1][0] * original_w)
                y1 = int(landmarks_normalized[idx1][1] * original_h)
                x2 = int(landmarks_normalized[idx2][0] * original_w)
                y2 = int(landmarks_normalized[idx2][1] * original_h)
                
                cv2.line(result_image, (x1, y1), (x2, y2), color, 1)
    
    # Draw key landmark points larger (eyes, nose, mouth corners)
    key_points = [
        (1, (0, 0, 255)),      # Nose tip - red
        (33, (255, 0, 0)),     # Right eye inner - blue
        (133, (255, 0, 0)),    # Right eye outer - blue
        (362, (255, 0, 0)),    # Left eye outer - blue
        (263, (255, 0, 0)),    # Left eye inner - blue
        (61, (0, 255, 255)),   # Mouth right - yellow
        (291, (0, 255, 255)),  # Mouth left - yellow
        (17, (0, 255, 255)),   # Mouth bottom - yellow
        (0, (0, 255, 255)),    # Mouth top - yellow
    ]
    
    for idx, color in key_points:
        if idx < len(landmarks_normalized):
            px = int(landmarks_normalized[idx][0] * original_w)
            py = int(landmarks_normalized[idx][1] * original_h)
            cv2.circle(result_image, (px, py), 3, color, -1)
    
    # Add info text
    cv2.putText(result_image, f"468 Face Landmarks", (10, 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(result_image, f"Confidence: {float(confidence_output.flatten()[0]):.2f}", (10, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    # Save result
    print(f"\nSaving result to: {output_path}")
    cv2.imwrite(output_path, result_image)
    
    print("\n" + "="*60)
    print("✅ DONE!")
    print("="*60)
    print(f"\nResult saved to: {output_path}")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

