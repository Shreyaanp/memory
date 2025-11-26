#!/usr/bin/env python3
"""
Hybrid Face Tracking Test
- MediaPipe: Face detection (works reliably)  
- BPU: Face landmarks (hardware accelerated)

This combines the best of both worlds.
"""

import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import math
import time
from collections import deque

try:
    from hobot_dnn import pyeasy_dnn as dnn
    BPU_AVAILABLE = True
except ImportError:
    BPU_AVAILABLE = False
    print("Warning: hobot_dnn not available, using MediaPipe landmarks only")

# ============================================================================
# CONFIGURATION
# ============================================================================

WINDOW_NAME = "MDai Hybrid Face Tracking"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30

# Face landmarks model
FACE_LANDMARK_MODEL = "/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm"

# Validation thresholds
MIN_DISTANCE_CM = 25.0
MAX_DISTANCE_CM = 60.0
REFERENCE_FACE_WIDTH_PX = 150.0
REFERENCE_DISTANCE_CM = 40.0
EXTREME_YAW_THRESHOLD = 0.75

# Spiral tracking
SPIRAL_COMPLETE_ANGLE = 2.0 * math.pi

# Colors (BGR)
COLOR_GREEN = (0, 255, 136)
COLOR_RED = (0, 0, 255)
COLOR_GREY = (100, 100, 100)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_CYAN = (255, 255, 0)
COLOR_MAGENTA = (255, 0, 255)

# ============================================================================
# BPU FACE LANDMARKS (106 points)
# ============================================================================

class BPUFaceLandmarks:
    """106-point face landmarks using BPU"""
    
    def __init__(self, model_path):
        if not BPU_AVAILABLE:
            raise RuntimeError("BPU not available")
        
        print(f"Loading BPU face landmarks: {model_path}")
        self.models = dnn.load(model_path)
        self.model = self.models[0]
        self.input_size = 128
        print(f"  ✓ BPU landmarks model loaded (106 points)")
        
    def detect(self, face_crop, bbox_in_original):
        """
        Detect 106 landmarks in cropped face.
        Returns list of (x, y) in original image coordinates.
        """
        x1, y1, x2, y2 = bbox_in_original
        face_w = x2 - x1
        face_h = y2 - y1
        
        # Preprocess: resize to 128x128, BGR->RGB, NCHW
        resized = cv2.resize(face_crop, (self.input_size, self.input_size))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        nchw = np.transpose(rgb, (2, 0, 1))
        nchw = np.expand_dims(nchw, axis=0).astype(np.uint8)
        
        # Inference
        outputs = self.model.forward(nchw)
        
        # Parse landmarks
        landmarks = []
        try:
            out0 = outputs[0].buffer.flatten()  # x coords
            out1 = outputs[1].buffer.flatten()  # y coords
            
            # Get scale factors if available
            scale0 = outputs[0].properties.scale_data
            scale1 = outputs[1].properties.scale_data
            
            for i in range(106):
                # Decode quantized values
                if scale0 is not None and len(scale0) > 0:
                    x_norm = float(out0[i]) * float(scale0[0])
                else:
                    x_norm = float(out0[i]) / 128.0
                    
                if scale1 is not None and len(scale1) > 0:
                    y_norm = float(out1[i]) * float(scale1[0])
                else:
                    y_norm = float(out1[i]) / 128.0
                
                # Clamp to 0-1 range
                x_norm = max(0, min(1, x_norm))
                y_norm = max(0, min(1, y_norm))
                
                # Map to original image coordinates
                x = x1 + x_norm * face_w
                y = y1 + y_norm * face_h
                landmarks.append((x, y))
                
        except Exception as e:
            print(f"BPU landmark error: {e}")
        
        return landmarks

# ============================================================================
# SPIRAL TRACKER
# ============================================================================

class SpiralTracker:
    def __init__(self):
        self.history = deque(maxlen=60)
        self.cumulative_angle = 0.0
        self.last_angle = None
        self.center_x = 0.5
        self.center_y = 0.5
        
    def update(self, nose_x, nose_y, is_valid):
        if not is_valid:
            return self.cumulative_angle / SPIRAL_COMPLETE_ANGLE
        
        self.history.append((nose_x, nose_y))
        if len(self.history) < 5:
            return 0.0
        
        self.center_x = sum(p[0] for p in self.history) / len(self.history)
        self.center_y = sum(p[1] for p in self.history) / len(self.history)
        
        dx = nose_x - self.center_x
        dy = nose_y - self.center_y
        current_angle = math.atan2(dy, dx)
        
        if self.last_angle is not None:
            delta = current_angle - self.last_angle
            if delta > math.pi:
                delta -= 2 * math.pi
            elif delta < -math.pi:
                delta += 2 * math.pi
            if abs(delta) > 0.01:
                self.cumulative_angle += abs(delta)
        
        self.last_angle = current_angle
        return min(self.cumulative_angle / SPIRAL_COMPLETE_ANGLE, 1.0)
    
    def reset(self):
        self.history.clear()
        self.cumulative_angle = 0.0
        self.last_angle = None

# ============================================================================
# FACE VALIDATION  
# ============================================================================

def get_face_bbox_from_mediapipe(face_landmarks, img_w, img_h):
    """Extract face bounding box from MediaPipe landmarks"""
    xs = [lm.x * img_w for lm in face_landmarks.landmark]
    ys = [lm.y * img_h for lm in face_landmarks.landmark]
    
    x1 = max(0, int(min(xs)) - 20)
    y1 = max(0, int(min(ys)) - 20)
    x2 = min(img_w, int(max(xs)) + 20)
    y2 = min(img_h, int(max(ys)) + 20)
    
    return x1, y1, x2, y2

def estimate_face_distance(face_width_px):
    if face_width_px <= 0:
        return 0.0
    return (REFERENCE_FACE_WIDTH_PX * REFERENCE_DISTANCE_CM) / face_width_px

def is_orientation_extreme(landmarks, img_w, img_h):
    """Check if face has extreme rotation"""
    nose = landmarks.landmark[4]  # Nose tip
    left_eye = landmarks.landmark[33]
    right_eye = landmarks.landmark[263]
    
    eye_center_x = (left_eye.x + right_eye.x) / 2
    nose_offset = nose.x - eye_center_x
    eye_width = abs(right_eye.x - left_eye.x)
    yaw_ratio = abs(nose_offset / eye_width) if eye_width > 0 else 0
    
    return yaw_ratio > EXTREME_YAW_THRESHOLD, yaw_ratio

# ============================================================================
# DRAWING
# ============================================================================

def draw_landmarks_106(image, landmarks, color):
    """Draw 106-point BPU landmarks"""
    for i, (x, y) in enumerate(landmarks):
        cv2.circle(image, (int(x), int(y)), 2, color, -1)
    
    # Key points for 106-point model
    key_points = [46, 35, 83, 16]  # nose, left_eye, right_eye, chin
    for idx in key_points:
        if idx < len(landmarks):
            x, y = landmarks[idx]
            cv2.circle(image, (int(x), int(y)), 5, COLOR_CYAN, 2)

def draw_landmarks_468(image, mp_landmarks, color):
    """Draw MediaPipe 468 landmarks"""
    h, w = image.shape[:2]
    for lm in mp_landmarks.landmark:
        x, y = int(lm.x * w), int(lm.y * h)
        cv2.circle(image, (x, y), 1, color, -1)

def draw_info_panel(image, info):
    h, w = image.shape[:2]
    panel_x = w - 280
    cv2.rectangle(image, (panel_x, 0), (w, h), (30, 30, 30), -1)
    cv2.putText(image, "HYBRID TRACKING", (panel_x + 10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_CYAN, 2)
    
    y = 60
    for label, value, color in info:
        cv2.putText(image, f"{label}: {value}", (panel_x + 10, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        y += 22

# ============================================================================
# MAIN
# ============================================================================

def main():
    print("=" * 60)
    print("MDai Hybrid Face Tracking Test")
    print("MediaPipe (detection) + BPU (landmarks)")
    print("=" * 60)
    
    # Initialize MediaPipe
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        static_image_mode=False,
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    print("✓ MediaPipe Face Mesh initialized")
    
    # Initialize BPU landmarks (optional)
    bpu_landmarks = None
    if BPU_AVAILABLE:
        try:
            bpu_landmarks = BPUFaceLandmarks(FACE_LANDMARK_MODEL)
        except Exception as e:
            print(f"⚠ BPU landmarks unavailable: {e}")
    
    # Initialize camera
    pipeline = None
    cap = None
    
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) > 0:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, FPS)
            pipeline.start(config)
            print("✓ RealSense camera started")
        else:
            raise Exception("No RealSense")
    except Exception as e:
        print(f"RealSense: {e}, trying webcam...")
        for dev_id in range(6):
            cap = cv2.VideoCapture(dev_id)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    print(f"✓ Webcam {dev_id} started")
                    break
                cap.release()
        if cap is None or not cap.isOpened():
            print("No camera available")
            return
    
    # Initialize tracker
    spiral_tracker = SpiralTracker()
    
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1200, 700)
    
    print("\nControls: Q=Quit, R=Reset, B=Toggle BPU landmarks")
    print("=" * 60)
    
    use_bpu_landmarks = True
    fps_counter = 0
    fps_timer = time.time()
    fps = 0
    mp_time = 0
    bpu_time = 0
    
    try:
        while True:
            # Capture
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
            
            image = cv2.flip(image, 1)
            h, w = image.shape[:2]
            
            # FPS
            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                fps = fps_counter
                fps_counter = 0
                fps_timer = time.time()
            
            # MediaPipe detection
            t0 = time.time()
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = face_mesh.process(rgb)
            mp_time = (time.time() - t0) * 1000
            
            face_detected = False
            face_valid = False
            nose_x, nose_y = w // 2, h // 2
            distance_cm = 0.0
            yaw_ratio = 0.0
            progress = 0.0
            bpu_landmark_count = 0
            
            if results.multi_face_landmarks:
                face_detected = True
                mp_landmarks = results.multi_face_landmarks[0]
                
                # Get face bbox
                x1, y1, x2, y2 = get_face_bbox_from_mediapipe(mp_landmarks, w, h)
                face_width = x2 - x1
                
                # Draw MediaPipe landmarks (grey, background)
                draw_landmarks_468(image, mp_landmarks, COLOR_GREY)
                
                # Distance estimation
                distance_cm = estimate_face_distance(face_width)
                in_range = MIN_DISTANCE_CM <= distance_cm <= MAX_DISTANCE_CM
                
                # Orientation check
                is_extreme, yaw_ratio = is_orientation_extreme(mp_landmarks, w, h)
                
                face_valid = in_range and not is_extreme
                
                # Get nose from MediaPipe
                mp_nose = mp_landmarks.landmark[4]
                nose_x = mp_nose.x * w
                nose_y = mp_nose.y * h
                
                # BPU landmarks (if available and enabled)
                if bpu_landmarks and use_bpu_landmarks and face_valid:
                    face_crop = image[max(0,y1):min(h,y2), max(0,x1):min(w,x2)]
                    if face_crop.size > 0:
                        t0 = time.time()
                        bpu_lms = bpu_landmarks.detect(face_crop, (x1, y1, x2, y2))
                        bpu_time = (time.time() - t0) * 1000
                        
                        if bpu_lms and len(bpu_lms) >= 106:
                            bpu_landmark_count = len(bpu_lms)
                            draw_landmarks_106(image, bpu_lms, COLOR_GREEN)
                            
                            # Use BPU nose position (landmark 46)
                            nose_x, nose_y = bpu_lms[46]
                
                # Draw face bbox
                bbox_color = COLOR_GREEN if face_valid else COLOR_YELLOW
                cv2.rectangle(image, (x1, y1), (x2, y2), bbox_color, 2)
                
                # Update spiral
                progress = spiral_tracker.update(nose_x / w, nose_y / h, face_valid)
                
                # Draw nose target
                target_color = COLOR_GREEN if face_valid else COLOR_GREY
                cv2.circle(image, (int(nose_x), int(nose_y)), 15, target_color, 2)
                cv2.circle(image, (int(nose_x), int(nose_y)), 5, target_color, -1)
            
            # Info panel
            info = [
                ("Mode", "Hybrid (MP+BPU)" if bpu_landmarks else "MediaPipe only", COLOR_CYAN),
                ("Face", "YES" if face_detected else "NO", COLOR_GREEN if face_detected else COLOR_RED),
                ("Valid", "YES" if face_valid else "NO", COLOR_GREEN if face_valid else COLOR_RED),
                ("Distance", f"{distance_cm:.1f} cm", COLOR_GREEN if face_valid else COLOR_YELLOW),
                ("Yaw", f"{yaw_ratio:.2f}", COLOR_GREEN if yaw_ratio < EXTREME_YAW_THRESHOLD else COLOR_RED),
                ("", "", COLOR_GREY),
                ("Progress", f"{int(progress * 100)}%", COLOR_GREEN if progress >= 1.0 else COLOR_YELLOW),
                ("", "", COLOR_GREY),
                ("MP time", f"{mp_time:.1f}ms", COLOR_WHITE),
                ("BPU time", f"{bpu_time:.1f}ms" if bpu_landmark_count > 0 else "N/A", COLOR_WHITE),
                ("BPU pts", str(bpu_landmark_count), COLOR_MAGENTA if bpu_landmark_count > 0 else COLOR_GREY),
                ("FPS", str(fps), COLOR_CYAN),
                ("", "", COLOR_GREY),
                ("BPU lm", "ON" if use_bpu_landmarks else "OFF", COLOR_GREEN if use_bpu_landmarks else COLOR_RED),
            ]
            draw_info_panel(image, info)
            
            # Status bar
            if face_valid:
                status, color = "TRACKING", COLOR_GREEN
            elif face_detected:
                status, color = "FACE INVALID", COLOR_YELLOW
            else:
                status, color = "NO FACE", COLOR_RED
            
            cv2.rectangle(image, (0, h-40), (w-280, h), color, -1)
            cv2.putText(image, status, (20, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_WHITE, 2)
            
            cv2.imshow(WINDOW_NAME, image)
            
            key = cv2.waitKey(1) & 0xFF
            if key in [ord('q'), 27]:
                break
            elif key == ord('r'):
                spiral_tracker.reset()
                print("Reset")
            elif key == ord('b'):
                use_bpu_landmarks = not use_bpu_landmarks
                print(f"BPU landmarks: {'ON' if use_bpu_landmarks else 'OFF'}")
    
    except KeyboardInterrupt:
        pass
    
    finally:
        if pipeline:
            pipeline.stop()
        elif cap:
            cap.release()
        cv2.destroyAllWindows()
        face_mesh.close()
        print("\nDone")

if __name__ == "__main__":
    main()

