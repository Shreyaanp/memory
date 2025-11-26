#!/usr/bin/env python3
"""
Face Tracking GUI Test - MediaPipe Face Mesh Visualization
Test via FreeRDP to see face landmarks, distance, orientation
"""

import cv2
import numpy as np
import mediapipe as mp
import pyrealsense2 as rs
import math
import time
from collections import deque

# ============================================================================
# CONFIGURATION
# ============================================================================

WINDOW_NAME = "MDai Face Tracking Test"
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FPS = 30

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

# Key landmarks
NOSE_TIP = 4
LEFT_EYE_OUTER = 33
RIGHT_EYE_OUTER = 263
LEFT_EYE_INNER = 133
RIGHT_EYE_INNER = 362
CHIN = 152
FOREHEAD = 10

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
    cv2.line(image, (panel_x + 10, 40), (w - 10, 40), COLOR_GREY, 1)
    
    y = 70
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
    print("MDai Face Tracking GUI Test")
    print("=" * 60)
    print("\nInitializing...")
    
    # Initialize MediaPipe
    try:
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        static_image_mode=False,
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    print("✓ MediaPipe Face Mesh initialized")
    except Exception as e:
        print(f"✗ Failed to initialize MediaPipe: {e}")
        print("\nTroubleshooting:")
        print("  1. Check MediaPipe installation: pip3 install mediapipe")
        print("  2. Verify Python version: python3 --version (requires 3.8+)")
        print("  3. Check for GPU/OpenGL issues (MediaPipe will fallback to CPU)")
        return
    
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
            
            # Convert to RGB for MediaPipe
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
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
                # Run face detection
                try:
                results = face_mesh.process(rgb_image)
                except Exception as e:
                    print(f"MediaPipe processing error: {e}")
                    results = None
                
                if results.multi_face_landmarks:
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
            draw_info_panel(image, info)
            
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
            if cap:
            cap.release()
        cv2.destroyAllWindows()
        try:
            if 'face_mesh' in locals():
        face_mesh.close()
        except:
            pass
        print("\nCleanup complete")

if __name__ == "__main__":
    main()

