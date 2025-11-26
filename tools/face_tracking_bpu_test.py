#!/usr/bin/env python3
"""
Face Tracking BPU Test - Using Horizon BPU for Hardware Acceleration
This replaces MediaPipe with native BPU models for face detection and landmarks.

Models used:
1. multitask_body_head_face_hand_kps_960x544.hbm - Face detection
2. faceLandmark106pts.hbm - 106 point face landmarks
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import math
import time
from collections import deque

try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError:
    from hobot_dnn_rdkx5 import pyeasy_dnn as dnn

# ============================================================================
# CONFIGURATION
# ============================================================================

WINDOW_NAME = "MDai BPU Face Tracking Test"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30

# Model paths
FACE_DETECT_MODEL = "/opt/tros/humble/lib/mono2d_body_detection/config/multitask_body_head_face_hand_kps_960x544.hbm"
FACE_LANDMARK_MODEL = "/opt/tros/humble/share/face_landmarks_detection/config/faceLandmark106pts.hbm"

# Face validation thresholds
MIN_DISTANCE_CM = 25.0
MAX_DISTANCE_CM = 60.0
REFERENCE_FACE_WIDTH_PX = 200.0  # pixels at 40cm (adjusted for 960x544)
REFERENCE_DISTANCE_CM = 40.0

# Spiral tracking
SPIRAL_COMPLETE_ANGLE = 2.0 * math.pi

# Colors (BGR)
COLOR_GREEN = (0, 255, 136)
COLOR_RED = (0, 0, 255)
COLOR_GREY = (100, 100, 100)
COLOR_YELLOW = (0, 255, 255)
COLOR_WHITE = (255, 255, 255)
COLOR_CYAN = (255, 255, 0)

# 106 landmark indices for key points
# Based on Horizon's 106-point face landmark model
LANDMARK_NOSE_TIP = 46  # Nose tip
LANDMARK_LEFT_EYE = 35  # Left eye outer corner
LANDMARK_RIGHT_EYE = 83  # Right eye outer corner
LANDMARK_CHIN = 16  # Chin
LANDMARK_FOREHEAD = 72  # Forehead center

# ============================================================================
# BPU MODEL WRAPPER
# ============================================================================

class BPUFaceDetector:
    """Face detection using BPU multi-task model"""
    
    def __init__(self, model_path):
        print(f"Loading face detection model: {model_path}")
        self.models = dnn.load(model_path)
        self.model = self.models[0]
        
        # Get input shape
        input_props = self.model.inputs[0].properties
        self.input_shape = input_props.shape  # (1, 3, 544, 960) NCHW
        self.input_h = self.input_shape[2]
        self.input_w = self.input_shape[3]
        
        print(f"  Input: {self.input_w}x{self.input_h}")
        print(f"  Outputs: {len(self.model.outputs)}")
        
    def preprocess(self, bgr_image):
        """Convert BGR image to model input format (NV12 or resized RGB)"""
        # Resize to model input size
        resized = cv2.resize(bgr_image, (self.input_w, self.input_h))
        # Convert BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # Transpose to NCHW format
        nchw = np.transpose(rgb, (2, 0, 1))
        # Add batch dimension
        nchw = np.expand_dims(nchw, axis=0).astype(np.uint8)
        return nchw
    
    def detect(self, bgr_image):
        """
        Detect faces in image.
        Returns list of face bounding boxes: [(x1, y1, x2, y2, score), ...]
        """
        h, w = bgr_image.shape[:2]
        
        # Preprocess
        input_data = self.preprocess(bgr_image)
        
        # Run inference
        outputs = self.model.forward(input_data)
        
        # Parse face detections from outputs
        # Output indices for face detection (based on multitask model structure):
        # Outputs 4,5 are for face detection (class 2 in the multi-task model)
        faces = self._parse_detections(outputs, h, w, class_idx=2)
        
        return faces
    
    def _parse_detections(self, outputs, orig_h, orig_w, class_idx=2):
        """
        Parse detections from model outputs.
        The multi-task model outputs bboxes for body(0), head(1), face(2), hand(3).
        Each has 2 outputs: bbox regression and classification.
        """
        faces = []
        
        # The output structure for multi-task model:
        # [body_bbox, body_cls, head_bbox, head_cls, face_bbox, face_cls, hand_bbox, hand_cls, keypoints]
        # Face outputs are at indices 4 (bbox) and 5 (cls)
        
        try:
            # Get face bbox and classification outputs
            bbox_output_idx = class_idx * 2      # 4 for face
            cls_output_idx = class_idx * 2 + 1   # 5 for face
            
            bbox_data = outputs[bbox_output_idx].buffer
            cls_data = outputs[cls_output_idx].buffer
            
            # Scale factors
            scale_x = orig_w / self.input_w
            scale_y = orig_h / self.input_h
            
            # Parse detections (simplified - actual parsing depends on model specifics)
            # Assuming format similar to FCOS: [batch, 1, num_anchors, 5] for bbox
            if len(bbox_data.shape) == 4:
                num_detections = bbox_data.shape[2]
                
                for i in range(min(num_detections, 10)):  # Max 10 faces
                    # Get confidence (assuming it's in cls_data)
                    score = float(cls_data[0, 0, i, 0]) if len(cls_data.shape) == 4 else 0.5
                    
                    if score > 0.3:  # Confidence threshold
                        # Get bbox coordinates
                        x1 = float(bbox_data[0, 0, i, 0]) * scale_x
                        y1 = float(bbox_data[0, 0, i, 1]) * scale_y
                        x2 = float(bbox_data[0, 0, i, 2]) * scale_x
                        y2 = float(bbox_data[0, 0, i, 3]) * scale_y
                        
                        if x2 > x1 and y2 > y1:  # Valid bbox
                            faces.append((int(x1), int(y1), int(x2), int(y2), score))
        
        except Exception as e:
            print(f"Detection parse error: {e}")
        
        return faces


class BPUFaceLandmarks:
    """106-point face landmarks using BPU"""
    
    def __init__(self, model_path):
        print(f"Loading face landmarks model: {model_path}")
        self.models = dnn.load(model_path)
        self.model = self.models[0]
        
        # Get input shape (1, 3, 128, 128)
        input_props = self.model.inputs[0].properties
        self.input_shape = input_props.shape
        self.input_size = self.input_shape[2]  # 128
        
        print(f"  Input: {self.input_size}x{self.input_size}")
        print(f"  Outputs: {len(self.model.outputs)} (106 landmarks)")
        
    def preprocess(self, face_crop):
        """Preprocess cropped face for landmark model"""
        # Resize to 128x128
        resized = cv2.resize(face_crop, (self.input_size, self.input_size))
        # BGR to RGB
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        # NCHW format
        nchw = np.transpose(rgb, (2, 0, 1))
        nchw = np.expand_dims(nchw, axis=0).astype(np.uint8)
        return nchw
    
    def detect(self, face_crop, bbox):
        """
        Detect 106 landmarks in cropped face.
        Returns list of (x, y) coordinates in original image space.
        """
        x1, y1, x2, y2, _ = bbox
        face_w = x2 - x1
        face_h = y2 - y1
        
        # Preprocess
        input_data = self.preprocess(face_crop)
        
        # Run inference
        outputs = self.model.forward(input_data)
        
        # Parse landmarks
        # Output shape: (1, 1, 32, 106) or similar - need to decode
        landmarks = []
        
        try:
            # Get raw output data
            out0 = outputs[0].buffer  # x coordinates
            out1 = outputs[1].buffer  # y coordinates
            
            # Decode quantized values
            scale0 = outputs[0].properties.scale_data if hasattr(outputs[0].properties, 'scale_data') else None
            scale1 = outputs[1].properties.scale_data if hasattr(outputs[1].properties, 'scale_data') else None
            
            for i in range(106):
                # Get normalized coordinates (0-1)
                if scale0 is not None and len(scale0) > 0:
                    x_norm = float(out0.flatten()[i]) * float(scale0[0])
                else:
                    x_norm = float(out0.flatten()[i]) / 128.0
                    
                if scale1 is not None and len(scale1) > 0:
                    y_norm = float(out1.flatten()[i]) * float(scale1[0])
                else:
                    y_norm = float(out1.flatten()[i]) / 128.0
                
                # Map back to original image coordinates
                x = x1 + x_norm * face_w
                y = y1 + y_norm * face_h
                
                landmarks.append((x, y))
                
        except Exception as e:
            print(f"Landmark parse error: {e}")
        
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
# DRAWING FUNCTIONS
# ============================================================================

def draw_landmarks(image, landmarks, color):
    """Draw 106 face landmarks"""
    for i, (x, y) in enumerate(landmarks):
        cv2.circle(image, (int(x), int(y)), 2, color, -1)
    
    # Highlight key landmarks
    key_indices = [LANDMARK_NOSE_TIP, LANDMARK_LEFT_EYE, LANDMARK_RIGHT_EYE, LANDMARK_CHIN]
    for idx in key_indices:
        if idx < len(landmarks):
            x, y = landmarks[idx]
            cv2.circle(image, (int(x), int(y)), 5, COLOR_CYAN, 2)

def draw_face_bbox(image, bbox, color):
    """Draw face bounding box"""
    x1, y1, x2, y2, score = bbox
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    cv2.putText(image, f"{score:.2f}", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

def draw_info_panel(image, info):
    """Draw info panel"""
    h, w = image.shape[:2]
    panel_x = w - 280
    cv2.rectangle(image, (panel_x, 0), (w, h), (30, 30, 30), -1)
    
    cv2.putText(image, "BPU FACE TRACKING", (panel_x + 10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_CYAN, 2)
    
    y = 60
    for label, value, color in info:
        cv2.putText(image, f"{label}: {value}", (panel_x + 10, y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        y += 25


# ============================================================================
# MAIN APPLICATION
# ============================================================================

def main():
    print("=" * 60)
    print("MDai BPU Face Tracking Test")
    print("Using Horizon BPU Hardware Acceleration")
    print("=" * 60)
    print()
    
    # Initialize BPU models
    print("Loading BPU models...")
    try:
        face_detector = BPUFaceDetector(FACE_DETECT_MODEL)
        face_landmarks = BPUFaceLandmarks(FACE_LANDMARK_MODEL)
    except Exception as e:
        print(f"Failed to load BPU models: {e}")
        return
    
    print("\n✓ BPU models loaded successfully!")
    
    # Initialize camera
    print("\nInitializing camera...")
    pipeline = None
    cap = None
    
    try:
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) > 0:
            print(f"  Found: {devices[0].get_info(rs.camera_info.name)}")
            
            pipeline = rs.pipeline()
            config = rs.config()
            
            # Try common resolutions
            resolutions = [
                (960, 540, 30),
                (640, 480, 30),
                (1280, 720, 15),
            ]
            
            started = False
            for w, h, fps in resolutions:
                try:
                    config = rs.config()
                    config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
                    pipeline.start(config)
                    print(f"✓ RealSense camera started at {w}x{h}@{fps}fps")
                    started = True
                    break
                except Exception as res_err:
                    continue
            
            if not started:
                raise Exception("Could not start with any resolution")
        else:
            raise Exception("No RealSense device")
    except Exception as e:
        print(f"RealSense failed: {e}")
        print("Trying webcam...")
        pipeline = None
        
        for dev_id in range(6):
            cap = cv2.VideoCapture(dev_id)
            if cap.isOpened():
                ret, test_frame = cap.read()
                if ret and test_frame is not None:
                    print(f"✓ Webcam /dev/video{dev_id} started")
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    break
                cap.release()
            cap = None
        
        if cap is None:
            print("Failed to open any camera")
            return
    
    # Initialize tracker
    spiral_tracker = SpiralTracker()
    
    # Create window
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)
    
    print("\n" + "=" * 60)
    print("CONTROLS: Q=Quit, R=Reset, SPACE=Pause")
    print("=" * 60)
    
    paused = False
    fps_counter = 0
    fps_timer = time.time()
    fps = 0
    detect_time = 0
    landmark_time = 0
    
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
            
            image = cv2.flip(image, 1)
            h, w = image.shape[:2]
            
            # FPS calculation
            fps_counter += 1
            if time.time() - fps_timer >= 1.0:
                fps = fps_counter
                fps_counter = 0
                fps_timer = time.time()
            
            # Default values
            face_detected = False
            face_valid = False
            nose_x, nose_y = w // 2, h // 2
            distance_cm = 0.0
            progress = 0.0
            landmarks = []
            
            if not paused:
                # Face detection
                t0 = time.time()
                faces = face_detector.detect(image)
                detect_time = (time.time() - t0) * 1000
                
                if faces:
                    face_detected = True
                    # Use largest face
                    faces.sort(key=lambda f: (f[2]-f[0]) * (f[3]-f[1]), reverse=True)
                    bbox = faces[0]
                    x1, y1, x2, y2, score = bbox
                    
                    # Draw face bbox
                    draw_face_bbox(image, bbox, COLOR_GREEN if face_valid else COLOR_YELLOW)
                    
                    # Extract face crop
                    face_crop = image[max(0,y1):min(h,y2), max(0,x1):min(w,x2)]
                    
                    if face_crop.size > 0:
                        # Face landmarks
                        t0 = time.time()
                        landmarks = face_landmarks.detect(face_crop, bbox)
                        landmark_time = (time.time() - t0) * 1000
                        
                        if landmarks and len(landmarks) >= 106:
                            # Draw landmarks
                            draw_landmarks(image, landmarks, COLOR_GREEN)
                            
                            # Get nose position
                            if LANDMARK_NOSE_TIP < len(landmarks):
                                nose_x, nose_y = landmarks[LANDMARK_NOSE_TIP]
                            
                            # Calculate distance
                            face_width = x2 - x1
                            distance_cm = (REFERENCE_FACE_WIDTH_PX * REFERENCE_DISTANCE_CM) / face_width
                            
                            # Validate face
                            face_valid = MIN_DISTANCE_CM <= distance_cm <= MAX_DISTANCE_CM
                            
                            # Update spiral tracking
                            progress = spiral_tracker.update(nose_x / w, nose_y / h, face_valid)
                            
                            # Draw nose target
                            color = COLOR_GREEN if face_valid else COLOR_GREY
                            cv2.circle(image, (int(nose_x), int(nose_y)), 15, color, 2)
                            cv2.circle(image, (int(nose_x), int(nose_y)), 5, color, -1)
            
            # Draw info panel
            info = [
                ("Mode", "BPU Accelerated", COLOR_CYAN),
                ("Face", "YES" if face_detected else "NO", COLOR_GREEN if face_detected else COLOR_RED),
                ("Valid", "YES" if face_valid else "NO", COLOR_GREEN if face_valid else COLOR_RED),
                ("Distance", f"{distance_cm:.1f} cm", COLOR_GREEN if face_valid else COLOR_YELLOW),
                ("Landmarks", str(len(landmarks)), COLOR_WHITE),
                ("Progress", f"{int(progress * 100)}%", COLOR_GREEN if progress >= 1.0 else COLOR_YELLOW),
                ("", "", COLOR_GREY),
                ("FPS", str(fps), COLOR_CYAN),
                ("Detect", f"{detect_time:.1f}ms", COLOR_WHITE),
                ("Landmark", f"{landmark_time:.1f}ms", COLOR_WHITE),
            ]
            draw_info_panel(image, info)
            
            # Status bar
            if face_valid:
                status = "TRACKING"
                status_color = COLOR_GREEN
            elif face_detected:
                status = "FACE INVALID"
                status_color = COLOR_YELLOW
            else:
                status = "NO FACE"
                status_color = COLOR_RED
            
            cv2.rectangle(image, (0, h-50), (w-280, h), status_color, -1)
            cv2.putText(image, status, (20, h-15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, COLOR_WHITE, 2)
            
            # Show
            cv2.imshow(WINDOW_NAME, image)
            
            # Keyboard
            key = cv2.waitKey(1) & 0xFF
            if key in [ord('q'), ord('Q'), 27]:
                break
            elif key in [ord('r'), ord('R')]:
                spiral_tracker.reset()
                print("Progress reset")
            elif key == ord(' '):
                paused = not paused
                print("Paused" if paused else "Resumed")
    
    except KeyboardInterrupt:
        print("\nInterrupted")
    
    finally:
        if pipeline:
            pipeline.stop()
        elif cap:
            cap.release()
        cv2.destroyAllWindows()
        print("\nDone")


if __name__ == "__main__":
    main()

