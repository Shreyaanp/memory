#!/usr/bin/env python3
"""
Face Tracking GUI Test - TFLite Face Mesh Visualization
Replaces MediaPipe with face_landmark.tflite model
Test via FreeRDP to see face landmarks, distance, orientation
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import math
import time
from collections import deque

# Try to import tflite_runtime first, fall back to tensorflow
try:
    from tflite_runtime.interpreter import Interpreter
    print("Using tflite_runtime")
except ImportError:
    try:
        import tensorflow as tf
        Interpreter = tf.lite.Interpreter
        print("Using tensorflow.lite")
    except ImportError:
        print("ERROR: Neither tflite_runtime nor tensorflow is installed!")
        print("Install with: pip install tflite-runtime")
        print("Or: pip install tensorflow")
        exit(1)

# ============================================================================
# CONFIGURATION
# ============================================================================

WINDOW_NAME = "MDai Face Tracking Test (TFLite)"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30

# TFLite model path
TFLITE_MODEL_PATH = "/home/mercleDev/codebase/tools/face_landmark.tflite"

# Face validation thresholds
MIN_DISTANCE_CM = 25.0
MAX_DISTANCE_CM = 60.0
REFERENCE_FACE_WIDTH_PX = 150.0  # pixels at 40cm
REFERENCE_DISTANCE_CM = 40.0
EXTREME_YAW_THRESHOLD = 0.75  # 75% toward eye = extreme

# Spiral tracking
SPIRAL_HISTORY_SIZE = 60
SPIRAL_COMPLETE_ANGLE = 2.0 * math.pi  # 360 degrees

# Colors (BGR for OpenCV)
COLOR_GREEN = (0, 255, 136)  # #00FF88
COLOR_RED = (0, 0, 255)
COLOR_GREY = (100, 100, 100)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_CYAN = (255, 255, 0)

# Key landmarks (MediaPipe 468-point topology)
NOSE_TIP = 4
LEFT_EYE_OUTER = 33
RIGHT_EYE_OUTER = 263
LEFT_EYE_INNER = 133
RIGHT_EYE_INNER = 362
CHIN = 152
FOREHEAD = 10

# ============================================================================
# TFLITE FACE MESH DETECTOR
# ============================================================================

class Landmark:
    """Simple landmark class to mimic MediaPipe's landmark structure"""
    def __init__(self, x, y, z=0.0):
        self.x = x
        self.y = y
        self.z = z

class TFLiteFaceMesh:
    """TFLite-based face mesh detector (replaces MediaPipe)"""
    
    def __init__(self, model_path):
        self.model_path = model_path
        self.interpreter = None
        self.input_details = None
        self.output_details = None
        self.face_cascade = None
        self.initialized = False
        
        # ROI tracking for stability
        self.last_roi = None
        self.roi_history = deque(maxlen=5)  # Track last 5 ROIs
        self.roi_stability_threshold = 0.3  # Max change ratio to consider stable
        self.roi_alpha = 0.7  # Smoothing factor for ROI (0.7 = 70% new, 30% old)
        
        # Landmark smoothing
        self.last_landmarks = None
        self.landmark_alpha = 0.6  # Smoothing factor for landmarks (0.6 = 60% new, 40% old)
        
        # Detection confidence tracking
        self.detection_fail_count = 0
        self.max_fail_count = 3  # Use previous ROI for 3 frames if detection fails
        
        # Try to use OpenCV DNN face detector (more stable than Haar cascade)
        self.face_dnn = None
        dnn_model_path = "/home/mercleDev/codebase/models/res10_300x300_ssd_iter_140000.caffemodel"
        dnn_config_path = "/home/mercleDev/codebase/models/deploy_ssd.prototxt"
        
        try:
            import os
            if os.path.exists(dnn_model_path) and os.path.exists(dnn_config_path):
                self.face_dnn = cv2.dnn.readNetFromCaffe(dnn_config_path, dnn_model_path)
                print("✓ Using OpenCV DNN face detector (more stable)")
            else:
                raise FileNotFoundError("DNN model files not found")
        except Exception as e:
            # Fallback to Haar cascade
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            if self.face_cascade.empty():
                print("⚠️ Warning: Could not load Haar cascade, will use full image")
            else:
                print("✓ Using Haar cascade face detector")
        
        # Load TFLite model
        try:
            self.interpreter = Interpreter(model_path=model_path)
            self.interpreter.allocate_tensors()
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            self.initialized = True
            print(f"✓ TFLite Face Mesh model loaded: {model_path}")
        except Exception as e:
            print(f"❌ Failed to load TFLite model: {e}")
            self.initialized = False
    
    def detect_face_roi(self, image):
        """Detect face ROI using DNN or Haar cascade with stability tracking"""
        h, w = image.shape[:2]
        
        # Try DNN first (more stable)
        if self.face_dnn is not None:
            # Prepare blob for DNN (300x300 input)
            blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), [104, 117, 123], False, False)
            self.face_dnn.setInput(blob)
            detections = self.face_dnn.forward()
            
            # Find best detection
            best_confidence = 0.5
            best_face = None
            
            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                if confidence > best_confidence:
                    best_confidence = confidence
                    x1 = int(detections[0, 0, i, 3] * w)
                    y1 = int(detections[0, 0, i, 4] * h)
                    x2 = int(detections[0, 0, i, 5] * w)
                    y2 = int(detections[0, 0, i, 6] * h)
                    best_face = (x1, y1, x2 - x1, y2 - y1)
            
            if best_face is not None:
                x, y, w, h = best_face
                self.detection_fail_count = 0
            else:
                # Use previous ROI if available
                if self.last_roi is not None and self.detection_fail_count < self.max_fail_count:
                    self.detection_fail_count += 1
                    return self.last_roi
                return None
        else:
            # Fallback to Haar cascade
            if self.face_cascade is None or self.face_cascade.empty():
                # Fallback: use full image
                return (0, 0, w, h)
            
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Use more stable detection parameters
            faces = self.face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.05,  # Smaller steps = more stable
                minNeighbors=7,   # Higher = more stable, fewer false positives
                minSize=(80, 80), # Larger minimum = more stable
                flags=cv2.CASCADE_SCALE_IMAGE
            )
            
            if len(faces) == 0:
                # Use previous ROI if available and not too many failures
                if self.last_roi is not None and self.detection_fail_count < self.max_fail_count:
                    self.detection_fail_count += 1
                    return self.last_roi
                return None
            
            # Reset fail count on successful detection
            self.detection_fail_count = 0
            
            # Return largest face
            faces = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)
            x, y, w, h = faces[0]
        
        # Expand ROI slightly (1.3x)
        cx, cy = x + w // 2, y + h // 2
        new_w = int(w * 1.3)
        new_h = int(h * 1.3)
        img_h, img_w = image.shape[:2]
        
        new_x = max(0, cx - new_w // 2)
        new_y = max(0, cy - new_h // 2)
        new_w = min(new_w, img_w - new_x)
        new_h = min(new_h, img_h - new_y)
        
        new_roi = (new_x, new_y, new_w, new_h)
        
        # Smooth ROI with previous ROI for stability
        if self.last_roi is not None:
            lx, ly, lw, lh = self.last_roi
            # Calculate center and size differences
            old_cx, old_cy = lx + lw // 2, ly + lh // 2
            new_cx, new_cy = new_x + new_w // 2, new_y + new_h // 2
            
            # Smooth center position
            smooth_cx = int(self.roi_alpha * new_cx + (1 - self.roi_alpha) * old_cx)
            smooth_cy = int(self.roi_alpha * new_cy + (1 - self.roi_alpha) * old_cy)
            
            # Smooth size
            smooth_w = int(self.roi_alpha * new_w + (1 - self.roi_alpha) * lw)
            smooth_h = int(self.roi_alpha * new_h + (1 - self.roi_alpha) * lh)
            
            # Recalculate bounds
            smooth_x = max(0, smooth_cx - smooth_w // 2)
            smooth_y = max(0, smooth_cy - smooth_h // 2)
            smooth_w = min(smooth_w, img_w - smooth_x)
            smooth_h = min(smooth_h, img_h - smooth_y)
            
            new_roi = (smooth_x, smooth_y, smooth_w, smooth_h)
        
        # Update tracking
        self.last_roi = new_roi
        self.roi_history.append(new_roi)
        
        return new_roi
    
    def process(self, image):
        """Process image and return landmarks (MediaPipe-like interface)"""
        if not self.initialized:
            return None
        
        # Detect face ROI
        roi = self.detect_face_roi(image)
        if roi is None:
            return None
        
        rx, ry, rw, rh = roi
        
        # Crop face region
        face_crop = image[ry:ry+rh, rx:rx+rw]
        
        # Preprocess for TFLite model (192x192, RGB, float32, normalized)
        face_rgb = cv2.cvtColor(face_crop, cv2.COLOR_BGR2RGB)
        face_resized = cv2.resize(face_rgb, (192, 192))
        face_normalized = face_resized.astype(np.float32) / 255.0
        face_input = np.expand_dims(face_normalized, axis=0)
        
        # Run inference
        self.interpreter.set_tensor(self.input_details[0]['index'], face_input)
        self.interpreter.invoke()
        
        # Get outputs
        landmarks_raw = self.interpreter.get_tensor(self.output_details[0]['index'])
        confidence = self.interpreter.get_tensor(self.output_details[1]['index'])
        
        # Check confidence
        conf_value = confidence.flatten()[0]
        if conf_value < 0.5:  # Low confidence threshold
            return None
        
        # Parse landmarks (1404 values = 468 * 3)
        landmarks_flat = landmarks_raw.flatten()
        if len(landmarks_flat) != 1404:
            print(f"⚠️ Unexpected landmark count: {len(landmarks_flat)}")
            return None
        
        landmarks_3d = landmarks_flat.reshape(468, 3)
        
        # Convert landmarks from 192x192 space to original image space
        h, w = image.shape[:2]
        landmarks = []
        
        for lx, ly, lz in landmarks_3d:
            # Convert from 192x192 space to ROI space
            roi_x = (lx / 192.0) * rw
            roi_y = (ly / 192.0) * rh
            
            # Convert from ROI space to original image space (normalized 0-1)
            img_x = (rx + roi_x) / w
            img_y = (ry + roi_y) / h
            
            landmarks.append(Landmark(img_x, img_y, lz))
        
        # Apply temporal smoothing to landmarks
        if self.last_landmarks is not None and len(self.last_landmarks) == len(landmarks):
            smoothed_landmarks = []
            for i, (new_lm, old_lm) in enumerate(zip(landmarks, self.last_landmarks)):
                # Smooth x, y coordinates
                smooth_x = self.landmark_alpha * new_lm.x + (1 - self.landmark_alpha) * old_lm.x
                smooth_y = self.landmark_alpha * new_lm.y + (1 - self.landmark_alpha) * old_lm.y
                # Z coordinate less smoothing (depth changes faster)
                smooth_z = 0.8 * new_lm.z + 0.2 * old_lm.z
                smoothed_landmarks.append(Landmark(smooth_x, smooth_y, smooth_z))
            landmarks = smoothed_landmarks
        
        # Update last landmarks
        self.last_landmarks = landmarks
        
        # Create result object similar to MediaPipe
        class Result:
            def __init__(self, landmarks):
                self.multi_face_landmarks = [FaceLandmarks(landmarks)]
        
        class FaceLandmarks:
            def __init__(self, landmarks):
                self.landmark = landmarks
        
        return Result(landmarks)
    
    def close(self):
        """Cleanup (for compatibility with MediaPipe interface)"""
        self.last_roi = None
        self.last_landmarks = None
        self.roi_history.clear()
        self.detection_fail_count = 0

# ============================================================================
# FACE VALIDATION
# ============================================================================

def estimate_face_distance(face_width_px):
    """Estimate face distance from camera using face width"""
    if face_width_px <= 0:
        return 0.0
    distance = (REFERENCE_FACE_WIDTH_PX * REFERENCE_DISTANCE_CM) / face_width_px
    return distance

def get_face_width(landmarks, img_width):
    """Get face width in pixels from ear-to-ear landmarks"""
    left = landmarks[234]  # Left ear
    right = landmarks[454]  # Right ear
    width = abs(right.x - left.x) * img_width
    return width

def is_face_orientation_extreme(landmarks, img_width, img_height):
    """
    Check if face has extreme yaw/pitch/roll
    Returns: (is_extreme, yaw_ratio, pitch_ratio, roll_degrees)
    """
    # Get key points
    nose = landmarks[NOSE_TIP]
    left_eye = landmarks[LEFT_EYE_OUTER]
    right_eye = landmarks[RIGHT_EYE_OUTER]
    chin = landmarks[CHIN]
    forehead = landmarks[FOREHEAD]
    
    # Calculate yaw (left-right head turn)
    eye_center_x = (left_eye.x + right_eye.x) / 2
    nose_offset = nose.x - eye_center_x
    eye_width = abs(right_eye.x - left_eye.x)
    yaw_ratio = abs(nose_offset / eye_width) if eye_width > 0 else 0
    
    # Calculate pitch (up-down head tilt)
    face_height = abs(forehead.y - chin.y)
    nose_y_ratio = (nose.y - forehead.y) / face_height if face_height > 0 else 0.5
    pitch_ratio = abs(nose_y_ratio - 0.5) * 2  # 0 = centered, 1 = extreme
    
    # Calculate roll (head tilt sideways)
    dx = (right_eye.x - left_eye.x) * img_width
    dy = (right_eye.y - left_eye.y) * img_height
    roll_degrees = math.degrees(math.atan2(dy, dx))
    
    is_extreme = (
        yaw_ratio > EXTREME_YAW_THRESHOLD or
        pitch_ratio > 0.6 or
        abs(roll_degrees) > 25
    )
    
    return is_extreme, yaw_ratio, pitch_ratio, roll_degrees

# ============================================================================
# SPIRAL MOTION TRACKER
# ============================================================================

class SpiralTracker:
    def __init__(self):
        self.history = deque(maxlen=SPIRAL_HISTORY_SIZE)
        self.cumulative_angle = 0.0
        self.last_angle = None
        self.center_x = 0.5
        self.center_y = 0.5
        
    def update(self, nose_x, nose_y, is_valid):
        """Update spiral tracking with new nose position"""
        if not is_valid:
            return self.cumulative_angle / SPIRAL_COMPLETE_ANGLE
        
        self.history.append((nose_x, nose_y))
        
        if len(self.history) < 5:
            return 0.0
        
        # Calculate dynamic center
        self.center_x = sum(p[0] for p in self.history) / len(self.history)
        self.center_y = sum(p[1] for p in self.history) / len(self.history)
        
        # Calculate current angle from center
        dx = nose_x - self.center_x
        dy = nose_y - self.center_y
        current_angle = math.atan2(dy, dx)
        
        if self.last_angle is not None:
            # Calculate angular delta
            delta = current_angle - self.last_angle
            
            # Handle wrap-around
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            
            # Only count if movement is significant
            if abs(delta) > 0.01:
                self.cumulative_angle += abs(delta)
        
        self.last_angle = current_angle
        
        progress = min(self.cumulative_angle / SPIRAL_COMPLETE_ANGLE, 1.0)
        return progress
    
    def reset(self):
        self.history.clear()
        self.cumulative_angle = 0.0
        self.last_angle = None

# ============================================================================
# DRAWING FUNCTIONS
# ============================================================================

def draw_face_mesh(image, landmarks, face_valid):
    """Draw all 468 face landmarks"""
    h, w = image.shape[:2]
    color = COLOR_GREEN if face_valid else COLOR_GREY
    
    # Draw all landmarks as small dots
    for i, lm in enumerate(landmarks):
        x = int(lm.x * w)
        y = int(lm.y * h)
        cv2.circle(image, (x, y), 1, color, -1)
    
    # Highlight key landmarks
    key_landmarks = [NOSE_TIP, LEFT_EYE_OUTER, RIGHT_EYE_OUTER, CHIN, FOREHEAD]
    for idx in key_landmarks:
        lm = landmarks[idx]
        x = int(lm.x * w)
        y = int(lm.y * h)
        cv2.circle(image, (x, y), 5, COLOR_CYAN, 2)
    
    # Draw face outline (face oval)
    face_oval = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288,
                 397, 365, 379, 378, 400, 377, 152, 148, 176, 149, 150, 136,
                 172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109]
    
    for i in range(len(face_oval)):
        pt1_idx = face_oval[i]
        pt2_idx = face_oval[(i + 1) % len(face_oval)]
        pt1 = (int(landmarks[pt1_idx].x * w), int(landmarks[pt1_idx].y * h))
        pt2 = (int(landmarks[pt2_idx].x * w), int(landmarks[pt2_idx].y * h))
        cv2.line(image, pt1, pt2, COLOR_YELLOW, 1)

def draw_nose_target(image, landmarks, face_valid):
    """Draw target indicator on nose"""
    h, w = image.shape[:2]
    nose = landmarks[NOSE_TIP]
    x = int(nose.x * w)
    y = int(nose.y * h)
    
    color = COLOR_GREEN if face_valid else COLOR_GREY
    
    # Draw crosshair
    size = 20
    cv2.line(image, (x - size, y), (x + size, y), color, 2)
    cv2.line(image, (x, y - size), (x, y + size), color, 2)
    cv2.circle(image, (x, y), 15, color, 2)
    cv2.circle(image, (x, y), 5, color, -1)

def draw_spiral_progress(image, progress, center_x, center_y):
    """Draw spiral progress indicator"""
    h, w = image.shape[:2]
    cx = int(center_x * w)
    cy = int(center_y * h)
    
    # Draw center point
    cv2.circle(image, (cx, cy), 8, COLOR_YELLOW, 2)
    cv2.drawMarker(image, (cx, cy), COLOR_YELLOW, cv2.MARKER_CROSS, 20, 2)
    
    # Draw progress arc
    radius = 100
    start_angle = -90  # Start from top
    end_angle = start_angle + int(progress * 360)
    
    # Background circle
    cv2.ellipse(image, (cx, cy), (radius, radius), 0, 0, 360, COLOR_GREY, 2)
    
    # Progress arc
    if progress > 0:
        cv2.ellipse(image, (cx, cy), (radius, radius), 0, start_angle, end_angle, COLOR_GREEN, 4)

def draw_status_bar(image, status, status_color):
    """Draw status bar at bottom"""
    h, w = image.shape[:2]
    cv2.rectangle(image, (0, h - 50), (w - 300, h), status_color, -1)
    cv2.putText(image, status, (20, h - 15), 
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, COLOR_WHITE, 2)

def draw_info_panel(image, info):
    """Draw information panel on the side"""
    h, w = image.shape[:2]
    
    # Panel background
    panel_x = w - 300
    cv2.rectangle(image, (panel_x, 0), (w, h), (30, 30, 30), -1)
    cv2.rectangle(image, (panel_x, 0), (w, h), COLOR_GREY, 2)
    
    # Title
    cv2.putText(image, "FACE TRACKING TEST", (panel_x + 10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_CYAN, 2)
    cv2.putText(image, "(TFLite)", (panel_x + 10, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_CYAN, 1)
    cv2.line(image, (panel_x + 10, 55), (w - 10, 55), COLOR_GREY, 1)
    
    y = 80
    line_height = 30
    
    for label, value, color in info:
        cv2.putText(image, label, (panel_x + 10, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_WHITE, 1)
        cv2.putText(image, str(value), (panel_x + 150, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y += line_height

# ============================================================================
# MAIN APPLICATION
# ============================================================================

def main():
    print("=" * 60)
    print("MDai Face Tracking GUI Test (TFLite)")
    print("=" * 60)
    print("\nInitializing...")
    
    # Initialize TFLite Face Mesh
    face_mesh = TFLiteFaceMesh(TFLITE_MODEL_PATH)
    if not face_mesh.initialized:
        print("❌ Failed to initialize TFLite Face Mesh")
        return
    print("✓ TFLite Face Mesh initialized")
    
    # Initialize RealSense
    pipeline = None
    cap = None
    
    try:
        # List available RealSense devices
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) > 0:
            print(f"Found {len(devices)} RealSense device(s)")
            for dev in devices:
                print(f"  - {dev.get_info(rs.camera_info.name)}: {dev.get_info(rs.camera_info.serial_number)}")
            
            # Try to hardware reset the camera first
            try:
                devices[0].hardware_reset()
                print("  Hardware reset performed, waiting...")
                time.sleep(2)
            except Exception as reset_err:
                print(f"  Reset skipped: {reset_err}")
            
            pipeline = rs.pipeline()
            config = rs.config()
            
            # Try different resolutions
            resolutions = [
                (CAMERA_WIDTH, CAMERA_HEIGHT, FPS),
                (640, 480, 30),
                (1280, 720, 15),
            ]
            
            started = False
            for w, h, fps in resolutions:
                try:
                    config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
                    pipeline.start(config)
                    print(f"✓ RealSense camera started at {w}x{h}@{fps}fps")
                    started = True
                    break
                except Exception as e:
                    config = rs.config()  # Reset config for next try
                    continue
            
            if not started:
                raise Exception("Could not start with any resolution")
        else:
            raise Exception("No RealSense devices found")
            
    except Exception as e:
        print(f"✗ RealSense failed: {e}")
        print("\nTrying regular webcam...")
        pipeline = None
        
        # Try different video devices
        for dev_id in range(6):
            cap = cv2.VideoCapture(dev_id)
            if cap.isOpened():
                ret, test_frame = cap.read()
                if ret and test_frame is not None:
                    print(f"✓ Camera /dev/video{dev_id} opened successfully")
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                    break
                cap.release()
            cap = None
        
        if cap is None:
            print("✗ No camera available")
            print("\n💡 Make sure camera is connected and not used by another process")
            return
    
    # Initialize spiral tracker
    spiral_tracker = SpiralTracker()
    
    # Create window
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)
    
    print("\n" + "=" * 60)
    print("CONTROLS:")
    print("  Q / ESC  - Quit")
    print("  R        - Reset spiral progress")
    print("  SPACE    - Pause/Resume")
    print("=" * 60)
    print("\nRunning... (view via FreeRDP)")
    
    paused = False
    frame_count = 0
    fps_timer = time.time()
    fps = 0
    
    try:
        while True:
            # Capture frame
            if pipeline:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                image = np.asanyarray(color_frame.get_data())
            else:
                ret, image = cap.read()
                if not ret:
                    continue
            
            # Flip for mirror effect
            image = cv2.flip(image, 1)
            h, w = image.shape[:2]
            
            # Calculate FPS
            frame_count += 1
            if time.time() - fps_timer >= 1.0:
                fps = frame_count
                frame_count = 0
                fps_timer = time.time()
            
            # Default values
            face_detected = False
            face_valid = False
            distance_cm = 0.0
            yaw_ratio = 0.0
            pitch_ratio = 0.0
            roll_deg = 0.0
            nose_x = 0.5
            nose_y = 0.5
            progress = 0.0
            status = "NO FACE DETECTED"
            status_color = COLOR_RED
            
            if not paused:
                # Run face detection with TFLite
                results = face_mesh.process(image)
                
                if results and results.multi_face_landmarks:
                    face_detected = True
                    landmarks = results.multi_face_landmarks[0].landmark
                    
                    # Get nose position
                    nose = landmarks[NOSE_TIP]
                    nose_x = nose.x
                    nose_y = nose.y
                    
                    # Calculate face width and distance
                    face_width = get_face_width(landmarks, w)
                    distance_cm = estimate_face_distance(face_width)
                    
                    # Check orientation
                    is_extreme, yaw_ratio, pitch_ratio, roll_deg = is_face_orientation_extreme(
                        landmarks, w, h
                    )
                    
                    # Validate face
                    in_range = MIN_DISTANCE_CM <= distance_cm <= MAX_DISTANCE_CM
                    face_valid = in_range and not is_extreme
                    
                    # Update spiral tracking
                    progress = spiral_tracker.update(nose_x, nose_y, face_valid)
                    
                    # Draw face mesh
                    draw_face_mesh(image, landmarks, face_valid)
                    draw_nose_target(image, landmarks, face_valid)
                    
                    # Determine status
                    if not in_range:
                        if distance_cm < MIN_DISTANCE_CM:
                            status = "TOO CLOSE - Move back"
                            status_color = (0, 165, 255)  # Orange
                        else:
                            status = "TOO FAR - Move closer"
                            status_color = (0, 165, 255)
                    elif is_extreme:
                        status = "FACE ROTATED - Look straight"
                        status_color = COLOR_YELLOW
                    else:
                        status = "TRACKING - Move in spiral"
                        status_color = COLOR_GREEN
                        
                        if progress >= 1.0:
                            status = "COMPLETE! Press R to reset"
                            status_color = COLOR_CYAN
            
            # Draw spiral progress
            draw_spiral_progress(image, progress, spiral_tracker.center_x, spiral_tracker.center_y)
            
            # Draw info panel
            info = [
                ("Face:", "DETECTED" if face_detected else "NONE", 
                 COLOR_GREEN if face_detected else COLOR_RED),
                ("Valid:", "YES" if face_valid else "NO",
                 COLOR_GREEN if face_valid else COLOR_RED),
                ("Distance:", f"{distance_cm:.1f} cm",
                 COLOR_GREEN if MIN_DISTANCE_CM <= distance_cm <= MAX_DISTANCE_CM else COLOR_RED),
                ("Range:", f"{MIN_DISTANCE_CM}-{MAX_DISTANCE_CM} cm", COLOR_GREY),
                ("", "", COLOR_GREY),
                ("Yaw:", f"{yaw_ratio:.2f}",
                 COLOR_GREEN if yaw_ratio <= EXTREME_YAW_THRESHOLD else COLOR_RED),
                ("Pitch:", f"{pitch_ratio:.2f}",
                 COLOR_GREEN if pitch_ratio <= 0.6 else COLOR_RED),
                ("Roll:", f"{roll_deg:.1f}°",
                 COLOR_GREEN if abs(roll_deg) <= 25 else COLOR_RED),
                ("", "", COLOR_GREY),
                ("Nose X:", f"{int(nose_x * w)}", COLOR_WHITE),
                ("Nose Y:", f"{int(nose_y * h)}", COLOR_WHITE),
                ("", "", COLOR_GREY),
                ("Progress:", f"{int(progress * 100)}%",
                 COLOR_GREEN if progress >= 1.0 else COLOR_YELLOW),
                ("Angle:", f"{math.degrees(spiral_tracker.cumulative_angle):.0f}°", COLOR_WHITE),
                ("", "", COLOR_GREY),
                ("FPS:", str(fps), COLOR_CYAN),
                ("Paused:", "YES" if paused else "NO", 
                 COLOR_YELLOW if paused else COLOR_GREEN),
            ]
            
            
            # Draw status bar
            draw_status_bar(image, status, status_color)
            
            # Show image
            cv2.imshow(WINDOW_NAME, image)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key in [ord('q'), ord('Q'), 27]:  # Q or ESC
                break
            elif key in [ord('r'), ord('R')]:
                spiral_tracker.reset()
                print("Spiral progress reset")
            elif key == ord(' '):
                paused = not paused
                print("Paused" if paused else "Resumed")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    
    finally:
        # Cleanup
        if pipeline:
            pipeline.stop()
        else:
            cap.release()
        cv2.destroyAllWindows()
        face_mesh.close()
        print("\nCleanup complete")

if __name__ == "__main__":
    main()

