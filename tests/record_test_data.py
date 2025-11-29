#!/usr/bin/env python3
"""
Test Data Recorder for MDAI System Testing
==========================================

Records RGB, Depth, and IR streams from RealSense camera along with
face detection data. Output is saved in a format that can be used
for comprehensive system testing.

Usage:
    python3 record_test_data.py --output test_face_scan.npz --duration 5

The recorded data includes:
- RGB frames (640x480 @ 30fps)
- Depth frames (aligned to color)
- IR frames
- Timestamps
- Face landmarks (if detected)
"""

import argparse
import numpy as np
import cv2
import time
import json
import os
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
import struct

try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    print("⚠️  pyrealsense2 not available - will use synthetic data generation")

try:
    import mediapipe as mp
    HAS_MEDIAPIPE = True
except ImportError:
    HAS_MEDIAPIPE = False
    print("⚠️  mediapipe not available - face landmarks won't be recorded")


@dataclass
class RecordedFrame:
    """Single frame of recorded data"""
    timestamp_ms: float
    frame_number: int
    rgb_data: np.ndarray  # Shape: (H, W, 3), dtype: uint8
    depth_data: np.ndarray  # Shape: (H, W), dtype: uint16
    ir_data: np.ndarray  # Shape: (H, W), dtype: uint8
    face_detected: bool = False
    face_bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)  # x, y, w, h
    landmarks: List[Tuple[float, float, float]] = field(default_factory=list)
    nose_x: float = 0.0
    nose_y: float = 0.0


class TestDataRecorder:
    """Records camera data for system testing"""
    
    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width = width
        self.height = height
        self.fps = fps
        self.frames: List[RecordedFrame] = []
        self.pipeline = None
        self.face_mesh = None
        self.recording = False
        
    def setup_camera(self) -> bool:
        """Initialize RealSense camera with same settings as production"""
        if not HAS_REALSENSE:
            print("❌ RealSense SDK not available")
            return False
            
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable streams (matching SystemController configuration)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(rs.stream.infrared, 1, self.width, self.height, rs.format.y8, self.fps)
            
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Get device and apply settings
            device = profile.get_device()
            
            # Configure depth sensor
            depth_sensor = device.first_depth_sensor()
            if depth_sensor.supports(rs.option.laser_power):
                depth_sensor.set_option(rs.option.laser_power, 150)  # Match production
            if depth_sensor.supports(rs.option.visual_preset):
                depth_sensor.set_option(rs.option.visual_preset, 3)  # High accuracy
                
            # Configure color sensor
            color_sensor = device.first_color_sensor()
            if color_sensor.supports(rs.option.enable_auto_exposure):
                color_sensor.set_option(rs.option.enable_auto_exposure, 1)
            if color_sensor.supports(rs.option.auto_exposure_priority):
                color_sensor.set_option(rs.option.auto_exposure_priority, 0)
                
            # Setup align to color
            self.align = rs.align(rs.stream.color)
            
            print(f"✅ Camera initialized: {self.width}x{self.height} @ {self.fps}fps")
            return True
            
        except Exception as e:
            print(f"❌ Camera setup failed: {e}")
            return False
            
    def setup_face_detection(self) -> bool:
        """Initialize MediaPipe face mesh"""
        if not HAS_MEDIAPIPE:
            return False
            
        try:
            self.mp_face_mesh = mp.solutions.face_mesh
            self.face_mesh = self.mp_face_mesh.FaceMesh(
                static_image_mode=False,
                max_num_faces=1,
                refine_landmarks=True,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
            print("✅ Face detection initialized (MediaPipe FaceMesh)")
            return True
        except Exception as e:
            print(f"⚠️  Face detection setup failed: {e}")
            return False
            
    def detect_face(self, rgb_frame: np.ndarray) -> Tuple[bool, List, Tuple[int, int, int, int], float, float]:
        """Detect face and extract landmarks"""
        if self.face_mesh is None:
            return False, [], (0, 0, 0, 0), 0.0, 0.0
            
        results = self.face_mesh.process(rgb_frame)
        
        if not results.multi_face_landmarks:
            return False, [], (0, 0, 0, 0), 0.0, 0.0
            
        landmarks = results.multi_face_landmarks[0]
        
        # Extract landmark coordinates
        h, w = rgb_frame.shape[:2]
        landmark_list = []
        xs, ys = [], []
        
        for lm in landmarks.landmark:
            x, y, z = lm.x * w, lm.y * h, lm.z
            landmark_list.append((x, y, z))
            xs.append(x)
            ys.append(y)
            
        # Calculate bounding box
        x_min, x_max = int(min(xs)), int(max(xs))
        y_min, y_max = int(min(ys)), int(max(ys))
        bbox = (x_min, y_min, x_max - x_min, y_max - y_min)
        
        # Nose tip (landmark 4)
        nose_x = landmark_list[4][0] / w
        nose_y = landmark_list[4][1] / h
        
        return True, landmark_list, bbox, nose_x, nose_y
        
    def record(self, duration_seconds: float, show_preview: bool = True) -> bool:
        """Record camera data for specified duration"""
        if self.pipeline is None:
            print("❌ Camera not initialized")
            return False
            
        print(f"\n🎬 Recording for {duration_seconds} seconds...")
        print("   Perform a slow spiral motion with your head")
        print("   Press 'q' to stop early\n")
        
        self.frames = []
        self.recording = True
        start_time = time.time()
        frame_count = 0
        
        try:
            while self.recording:
                elapsed = time.time() - start_time
                if elapsed >= duration_seconds:
                    break
                    
                # Wait for frames
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                aligned_frames = self.align.process(frames)
                
                # Get frames
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                ir_frame = frames.get_infrared_frame(1)
                
                if not color_frame or not depth_frame:
                    continue
                    
                # Convert to numpy
                rgb_data = np.asanyarray(color_frame.get_data())
                depth_data = np.asanyarray(depth_frame.get_data())
                ir_data = np.asanyarray(ir_frame.get_data()) if ir_frame else np.zeros((self.height, self.width), dtype=np.uint8)
                
                # Detect face
                face_detected, landmarks, bbox, nose_x, nose_y = self.detect_face(rgb_data)
                
                # Create frame record
                frame = RecordedFrame(
                    timestamp_ms=elapsed * 1000,
                    frame_number=frame_count,
                    rgb_data=rgb_data.copy(),
                    depth_data=depth_data.copy(),
                    ir_data=ir_data.copy(),
                    face_detected=face_detected,
                    face_bbox=bbox,
                    landmarks=landmarks,
                    nose_x=nose_x,
                    nose_y=nose_y
                )
                self.frames.append(frame)
                frame_count += 1
                
                # Show preview
                if show_preview:
                    preview = cv2.cvtColor(rgb_data, cv2.COLOR_RGB2BGR)
                    
                    # Draw face bbox
                    if face_detected:
                        x, y, w, h = bbox
                        cv2.rectangle(preview, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        
                        # Draw nose position
                        nose_px = int(nose_x * self.width)
                        nose_py = int(nose_y * self.height)
                        cv2.circle(preview, (nose_px, nose_py), 5, (0, 0, 255), -1)
                        
                    # Draw progress
                    progress = elapsed / duration_seconds
                    cv2.rectangle(preview, (10, 10), (10 + int(200 * progress), 30), (0, 255, 0), -1)
                    cv2.rectangle(preview, (10, 10), (210, 30), (255, 255, 255), 2)
                    
                    # Status text
                    status = f"Frame {frame_count} | Face: {'YES' if face_detected else 'NO'} | {elapsed:.1f}s/{duration_seconds}s"
                    cv2.putText(preview, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    cv2.imshow("Recording Test Data", preview)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("\n⏹️  Recording stopped by user")
                        break
                        
        except Exception as e:
            print(f"❌ Recording error: {e}")
            return False
        finally:
            self.recording = False
            if show_preview:
                cv2.destroyAllWindows()
                
        print(f"\n✅ Recorded {len(self.frames)} frames")
        faces_detected = sum(1 for f in self.frames if f.face_detected)
        print(f"   Face detected in {faces_detected}/{len(self.frames)} frames ({100*faces_detected/max(1,len(self.frames)):.1f}%)")
        
        return True
        
    def save(self, output_path: str) -> bool:
        """Save recorded data to file"""
        if not self.frames:
            print("❌ No frames to save")
            return False
            
        print(f"\n💾 Saving to {output_path}...")
        
        # Prepare arrays
        n_frames = len(self.frames)
        
        # Stack frame data
        rgb_stack = np.stack([f.rgb_data for f in self.frames])
        depth_stack = np.stack([f.depth_data for f in self.frames])
        ir_stack = np.stack([f.ir_data for f in self.frames])
        
        # Metadata arrays
        timestamps = np.array([f.timestamp_ms for f in self.frames])
        face_detected = np.array([f.face_detected for f in self.frames])
        face_bboxes = np.array([f.face_bbox for f in self.frames])
        nose_positions = np.array([[f.nose_x, f.nose_y] for f in self.frames])
        
        # Landmarks (variable length, so store as list of arrays)
        landmarks_list = [np.array(f.landmarks) if f.landmarks else np.array([]) for f in self.frames]
        
        # Save as compressed numpy archive
        np.savez_compressed(
            output_path,
            rgb_frames=rgb_stack,
            depth_frames=depth_stack,
            ir_frames=ir_stack,
            timestamps=timestamps,
            face_detected=face_detected,
            face_bboxes=face_bboxes,
            nose_positions=nose_positions,
            width=self.width,
            height=self.height,
            fps=self.fps,
            n_frames=n_frames
        )
        
        # Save landmarks separately (variable length)
        landmarks_path = output_path.replace('.npz', '_landmarks.npy')
        np.save(landmarks_path, np.array(landmarks_list, dtype=object), allow_pickle=True)
        
        # Calculate file size
        file_size = os.path.getsize(output_path) / (1024 * 1024)
        print(f"✅ Saved {n_frames} frames ({file_size:.1f} MB)")
        print(f"   Landmarks saved to: {landmarks_path}")
        
        return True
        
    def cleanup(self):
        """Release resources"""
        if self.pipeline:
            self.pipeline.stop()
        if self.face_mesh:
            self.face_mesh.close()
            
            
class SyntheticDataGenerator:
    """Generate synthetic test data when camera is not available"""
    
    @staticmethod
    def generate_spiral_motion(n_frames: int, width: int = 640, height: int = 480) -> List[RecordedFrame]:
        """Generate synthetic frames with spiral nose motion"""
        import math
        
        frames = []
        center_x, center_y = 0.5, 0.5
        radius = 0.15
        
        for i in range(n_frames):
            t = i / n_frames
            angle = t * 2 * math.pi  # One full rotation
            
            # Spiral motion
            nose_x = center_x + radius * math.cos(angle) * (1 - t * 0.3)
            nose_y = center_y + radius * math.sin(angle) * (1 - t * 0.3)
            
            # Generate synthetic RGB (gradient with face-like blob)
            rgb = np.zeros((height, width, 3), dtype=np.uint8)
            rgb[:, :, 0] = 100  # Some base color
            rgb[:, :, 1] = 80
            rgb[:, :, 2] = 70
            
            # Add "face" blob
            face_center = (int(nose_x * width), int(nose_y * height))
            cv2.circle(rgb, face_center, 80, (180, 150, 140), -1)
            
            # Generate synthetic depth (face closer than background)
            depth = np.ones((height, width), dtype=np.uint16) * 2000  # Background at 2m
            cv2.circle(depth, face_center, 80, 500, -1)  # Face at 0.5m
            
            # Generate synthetic IR
            ir = np.ones((height, width), dtype=np.uint8) * 50
            cv2.circle(ir, face_center, 80, 150, -1)
            
            # Synthetic landmarks (468 points in a grid around face)
            landmarks = []
            for j in range(468):
                lx = nose_x * width + (j % 22 - 11) * 5
                ly = nose_y * height + (j // 22 - 11) * 5
                landmarks.append((lx, ly, 0.0))
            
            # Face bbox
            bbox = (
                int(nose_x * width) - 80,
                int(nose_y * height) - 100,
                160,
                200
            )
            
            frame = RecordedFrame(
                timestamp_ms=i * (1000 / 30),  # 30 fps
                frame_number=i,
                rgb_data=rgb,
                depth_data=depth,
                ir_data=ir,
                face_detected=True,
                face_bbox=bbox,
                landmarks=landmarks,
                nose_x=nose_x,
                nose_y=nose_y
            )
            frames.append(frame)
            
        return frames
        
    @staticmethod
    def generate_no_face_data(n_frames: int, width: int = 640, height: int = 480) -> List[RecordedFrame]:
        """Generate frames with no face detected"""
        frames = []
        
        for i in range(n_frames):
            rgb = np.random.randint(50, 150, (height, width, 3), dtype=np.uint8)
            depth = np.random.randint(1000, 3000, (height, width), dtype=np.uint16)
            ir = np.random.randint(30, 100, (height, width), dtype=np.uint8)
            
            frame = RecordedFrame(
                timestamp_ms=i * (1000 / 30),
                frame_number=i,
                rgb_data=rgb,
                depth_data=depth,
                ir_data=ir,
                face_detected=False,
                face_bbox=(0, 0, 0, 0),
                landmarks=[],
                nose_x=0.0,
                nose_y=0.0
            )
            frames.append(frame)
            
        return frames
        
    @staticmethod
    def generate_spoof_data(n_frames: int, width: int = 640, height: int = 480) -> List[RecordedFrame]:
        """Generate frames that simulate a spoof attack (flat depth)"""
        frames = []
        
        for i in range(n_frames):
            # RGB looks like a face
            rgb = np.zeros((height, width, 3), dtype=np.uint8)
            rgb[:, :] = [180, 150, 140]  # Skin-like color
            
            # But depth is completely flat (photo/screen)
            depth = np.ones((height, width), dtype=np.uint16) * 800  # Flat at 0.8m
            
            # IR is also flat (no natural variance)
            ir = np.ones((height, width), dtype=np.uint8) * 100
            
            # Fake landmarks
            landmarks = [(width/2 + (j % 22 - 11) * 5, height/2 + (j // 22 - 11) * 5, 0.0) for j in range(468)]
            
            frame = RecordedFrame(
                timestamp_ms=i * (1000 / 30),
                frame_number=i,
                rgb_data=rgb,
                depth_data=depth,
                ir_data=ir,
                face_detected=True,
                face_bbox=(width//2 - 80, height//2 - 100, 160, 200),
                landmarks=landmarks,
                nose_x=0.5,
                nose_y=0.5
            )
            frames.append(frame)
            
        return frames


def main():
    parser = argparse.ArgumentParser(description="Record test data for MDAI system testing")
    parser.add_argument("--output", "-o", default="test_data/face_scan.npz", help="Output file path")
    parser.add_argument("--duration", "-d", type=float, default=5.0, help="Recording duration in seconds")
    parser.add_argument("--synthetic", "-s", action="store_true", help="Generate synthetic data instead of recording")
    parser.add_argument("--type", "-t", choices=["spiral", "no_face", "spoof"], default="spiral",
                        help="Type of synthetic data to generate")
    parser.add_argument("--frames", "-f", type=int, default=150, help="Number of frames for synthetic data")
    parser.add_argument("--no-preview", action="store_true", help="Disable preview window")
    parser.add_argument("--display", default=":10", help="X display to use (default: :10 for FreeRDP)")
    args = parser.parse_args()
    
    # Set display environment variable for FreeRDP
    import os
    if args.display:
        os.environ['DISPLAY'] = args.display
        print(f"📺 Using display: {args.display}")
    
    # Create output directory
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    
    if args.synthetic or not HAS_REALSENSE:
        print("\n🔧 Generating synthetic test data...")
        
        generator = SyntheticDataGenerator()
        
        if args.type == "spiral":
            frames = generator.generate_spiral_motion(args.frames)
            print(f"✅ Generated {len(frames)} frames with spiral motion")
        elif args.type == "no_face":
            frames = generator.generate_no_face_data(args.frames)
            print(f"✅ Generated {len(frames)} frames with no face")
        elif args.type == "spoof":
            frames = generator.generate_spoof_data(args.frames)
            print(f"✅ Generated {len(frames)} frames simulating spoof attack")
            
        # Save frames
        recorder = TestDataRecorder()
        recorder.frames = frames
        recorder.save(args.output)
        
    else:
        print("\n📹 Recording real camera data...")
        
        recorder = TestDataRecorder()
        
        if not recorder.setup_camera():
            print("❌ Failed to setup camera, generating synthetic data instead")
            frames = SyntheticDataGenerator.generate_spiral_motion(args.frames)
            recorder.frames = frames
            recorder.save(args.output)
            return
            
        recorder.setup_face_detection()
        
        if recorder.record(args.duration, show_preview=not args.no_preview):
            recorder.save(args.output)
            
        recorder.cleanup()
        
    print("\n✅ Done! Test data saved to:", args.output)
    print("\nUsage in tests:")
    print(f"  data = np.load('{args.output}')")
    print("  rgb_frames = data['rgb_frames']")
    print("  depth_frames = data['depth_frames']")
    print("  face_detected = data['face_detected']")


if __name__ == "__main__":
    main()

