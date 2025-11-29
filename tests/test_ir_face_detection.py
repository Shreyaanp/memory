#!/usr/bin/env python3
"""
IR → BGR Face Detection Test Suite
===================================
Tests the effectiveness of converting IR (grayscale) images to BGR
for use with MediaPipe Face Mesh / Face Landmarker.

Usage:
    # Test with RealSense camera (live)
    python test_ir_face_detection.py --live
    
    # Test with saved images
    python test_ir_face_detection.py --image path/to/ir_image.png
    
    # Run full benchmark with RealSense
    python test_ir_face_detection.py --benchmark --frames 100

Requirements:
    pip install mediapipe opencv-python numpy pyrealsense2
"""

import cv2
import numpy as np
import time
import argparse
from dataclasses import dataclass
from typing import Optional, Tuple, List
import os

# Optional imports
try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False
    print("⚠️  MediaPipe not installed: pip install mediapipe")

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("⚠️  pyrealsense2 not installed: pip install pyrealsense2")

try:
    import dlib
    DLIB_AVAILABLE = True
except ImportError:
    DLIB_AVAILABLE = False
    print("⚠️  dlib not installed (optional): pip install dlib")


@dataclass
class DetectionResult:
    """Result of a face detection attempt"""
    detected: bool
    confidence: float
    landmarks_count: int
    nose_position: Optional[Tuple[float, float]]
    bbox: Optional[Tuple[int, int, int, int]]  # x, y, w, h
    processing_time_ms: float
    method: str


class IRFaceDetectionTester:
    """Test suite for IR face detection with various conversion methods"""
    
    def __init__(self):
        self.mp_face_mesh = None
        self.mp_face_detection = None
        self.dlib_detector = None
        self.dlib_predictor = None
        self.opencv_cascade = None
        self.opencv_dnn = None
        
        self._init_detectors()
    
    def _init_detectors(self):
        """Initialize all available face detectors"""
        
        # MediaPipe Face Mesh (478 landmarks)
        if MEDIAPIPE_AVAILABLE:
            try:
                self.mp_face_mesh = mp.solutions.face_mesh.FaceMesh(
                    static_image_mode=False,
                    max_num_faces=1,
                    refine_landmarks=True,
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5
                )
                print("✅ MediaPipe Face Mesh initialized (478 landmarks)")
            except Exception as e:
                print(f"❌ MediaPipe Face Mesh failed: {e}")
            
            # MediaPipe Face Detection (faster, bbox only)
            try:
                self.mp_face_detection = mp.solutions.face_detection.FaceDetection(
                    model_selection=0,  # 0 = short-range, 1 = full-range
                    min_detection_confidence=0.5
                )
                print("✅ MediaPipe Face Detection initialized (bbox only)")
            except Exception as e:
                print(f"❌ MediaPipe Face Detection failed: {e}")
        
        # dlib (68 landmarks)
        if DLIB_AVAILABLE:
            try:
                self.dlib_detector = dlib.get_frontal_face_detector()
                # Check for predictor file
                predictor_paths = [
                    "shape_predictor_68_face_landmarks.dat",
                    "/usr/share/dlib/shape_predictor_68_face_landmarks.dat",
                    os.path.expanduser("~/shape_predictor_68_face_landmarks.dat"),
                ]
                for path in predictor_paths:
                    if os.path.exists(path):
                        self.dlib_predictor = dlib.shape_predictor(path)
                        print(f"✅ dlib initialized (68 landmarks) from {path}")
                        break
                if self.dlib_predictor is None:
                    print("⚠️  dlib detector loaded, but no landmark predictor found")
                    print("   Download from: http://dlib.net/files/shape_predictor_68_face_landmarks.dat.bz2")
            except Exception as e:
                print(f"❌ dlib failed: {e}")
        
        # OpenCV Haar Cascade
        try:
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.opencv_cascade = cv2.CascadeClassifier(cascade_path)
            if self.opencv_cascade.empty():
                raise Exception("Failed to load cascade")
            print("✅ OpenCV Haar Cascade initialized")
        except Exception as e:
            print(f"❌ OpenCV Haar Cascade failed: {e}")
        
        # OpenCV DNN (ResNet-SSD)
        try:
            model_path = "/home/mercleDev/codebase/models/res10_300x300_ssd_iter_140000.caffemodel"
            config_path = "/home/mercleDev/codebase/models/deploy_ssd.prototxt"
            if os.path.exists(model_path) and os.path.exists(config_path):
                self.opencv_dnn = cv2.dnn.readNetFromCaffe(config_path, model_path)
                print("✅ OpenCV DNN (ResNet-SSD) initialized")
            else:
                print("⚠️  OpenCV DNN models not found at expected paths")
        except Exception as e:
            print(f"❌ OpenCV DNN failed: {e}")
    
    # =========================================================================
    # IR → BGR Conversion Methods
    # =========================================================================
    
    def convert_simple(self, ir_gray: np.ndarray) -> np.ndarray:
        """Simple 3-channel conversion (just duplicate grayscale)"""
        return cv2.cvtColor(ir_gray, cv2.COLOR_GRAY2BGR)
    
    def convert_clahe(self, ir_gray: np.ndarray, clip_limit: float = 2.0) -> np.ndarray:
        """CLAHE (Contrast Limited Adaptive Histogram Equalization) + BGR"""
        clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
        enhanced = clahe.apply(ir_gray)
        return cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
    
    def convert_histogram_eq(self, ir_gray: np.ndarray) -> np.ndarray:
        """Global histogram equalization + BGR"""
        equalized = cv2.equalizeHist(ir_gray)
        return cv2.cvtColor(equalized, cv2.COLOR_GRAY2BGR)
    
    def convert_adaptive_threshold(self, ir_gray: np.ndarray) -> np.ndarray:
        """Adaptive thresholding for edge enhancement + original"""
        # Bilateral filter to reduce noise while keeping edges
        filtered = cv2.bilateralFilter(ir_gray, 9, 75, 75)
        # CLAHE
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(filtered)
        return cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
    
    def convert_gamma_correction(self, ir_gray: np.ndarray, gamma: float = 1.2) -> np.ndarray:
        """Gamma correction + BGR"""
        inv_gamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** inv_gamma) * 255 
                         for i in np.arange(0, 256)]).astype("uint8")
        corrected = cv2.LUT(ir_gray, table)
        return cv2.cvtColor(corrected, cv2.COLOR_GRAY2BGR)
    
    # =========================================================================
    # Detection Methods
    # =========================================================================
    
    def detect_mediapipe_mesh(self, bgr_image: np.ndarray) -> DetectionResult:
        """Detect face using MediaPipe Face Mesh (478 landmarks)"""
        if self.mp_face_mesh is None:
            return DetectionResult(False, 0, 0, None, None, 0, "mediapipe_mesh")
        
        start = time.perf_counter()
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        results = self.mp_face_mesh.process(rgb)
        elapsed = (time.perf_counter() - start) * 1000
        
        if results.multi_face_landmarks:
            landmarks = results.multi_face_landmarks[0]
            h, w = bgr_image.shape[:2]
            
            # Nose tip is landmark #1 in MediaPipe
            nose = landmarks.landmark[1]
            nose_pos = (nose.x * w, nose.y * h)
            
            # Calculate bounding box from landmarks
            xs = [lm.x * w for lm in landmarks.landmark]
            ys = [lm.y * h for lm in landmarks.landmark]
            bbox = (int(min(xs)), int(min(ys)), 
                   int(max(xs) - min(xs)), int(max(ys) - min(ys)))
            
            # Estimate confidence from visibility scores
            visibility_scores = [lm.visibility if hasattr(lm, 'visibility') else 1.0 
                               for lm in landmarks.landmark]
            confidence = np.mean(visibility_scores) if visibility_scores else 0.8
            
            return DetectionResult(
                detected=True,
                confidence=confidence,
                landmarks_count=len(landmarks.landmark),
                nose_position=nose_pos,
                bbox=bbox,
                processing_time_ms=elapsed,
                method="mediapipe_mesh"
            )
        
        return DetectionResult(False, 0, 0, None, None, elapsed, "mediapipe_mesh")
    
    def detect_mediapipe_face(self, bgr_image: np.ndarray) -> DetectionResult:
        """Detect face using MediaPipe Face Detection (bbox + 6 keypoints)"""
        if self.mp_face_detection is None:
            return DetectionResult(False, 0, 0, None, None, 0, "mediapipe_face")
        
        start = time.perf_counter()
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        results = self.mp_face_detection.process(rgb)
        elapsed = (time.perf_counter() - start) * 1000
        
        if results.detections:
            detection = results.detections[0]
            h, w = bgr_image.shape[:2]
            
            bbox_rel = detection.location_data.relative_bounding_box
            bbox = (
                int(bbox_rel.xmin * w),
                int(bbox_rel.ymin * h),
                int(bbox_rel.width * w),
                int(bbox_rel.height * h)
            )
            
            # Get nose position (keypoint index 2)
            keypoints = detection.location_data.relative_keypoints
            if len(keypoints) > 2:
                nose = keypoints[2]  # NOSE_TIP
                nose_pos = (nose.x * w, nose.y * h)
            else:
                nose_pos = None
            
            return DetectionResult(
                detected=True,
                confidence=detection.score[0],
                landmarks_count=6,  # 6 keypoints
                nose_position=nose_pos,
                bbox=bbox,
                processing_time_ms=elapsed,
                method="mediapipe_face"
            )
        
        return DetectionResult(False, 0, 0, None, None, elapsed, "mediapipe_face")
    
    def detect_dlib(self, bgr_image: np.ndarray) -> DetectionResult:
        """Detect face using dlib (68 landmarks)"""
        if self.dlib_detector is None:
            return DetectionResult(False, 0, 0, None, None, 0, "dlib")
        
        start = time.perf_counter()
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        faces = self.dlib_detector(gray, 0)
        elapsed = (time.perf_counter() - start) * 1000
        
        if len(faces) > 0:
            face = faces[0]
            bbox = (face.left(), face.top(), face.width(), face.height())
            
            nose_pos = None
            landmarks_count = 0
            
            if self.dlib_predictor:
                shape = self.dlib_predictor(gray, face)
                landmarks_count = shape.num_parts
                # Nose tip is landmark #30 in dlib 68-point model
                if landmarks_count >= 31:
                    nose = shape.part(30)
                    nose_pos = (float(nose.x), float(nose.y))
            
            return DetectionResult(
                detected=True,
                confidence=0.9,  # dlib doesn't provide confidence
                landmarks_count=landmarks_count,
                nose_position=nose_pos,
                bbox=bbox,
                processing_time_ms=elapsed,
                method="dlib"
            )
        
        return DetectionResult(False, 0, 0, None, None, elapsed, "dlib")
    
    def detect_opencv_cascade(self, bgr_image: np.ndarray) -> DetectionResult:
        """Detect face using OpenCV Haar Cascade (bbox only)"""
        if self.opencv_cascade is None:
            return DetectionResult(False, 0, 0, None, None, 0, "opencv_cascade")
        
        start = time.perf_counter()
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        faces = self.opencv_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
        )
        elapsed = (time.perf_counter() - start) * 1000
        
        if len(faces) > 0:
            x, y, w, h = faces[0]
            # Estimate nose position as center of face
            nose_pos = (x + w/2, y + h*0.6)  # Nose is ~60% down the face
            
            return DetectionResult(
                detected=True,
                confidence=0.8,
                landmarks_count=0,
                nose_position=nose_pos,
                bbox=(x, y, w, h),
                processing_time_ms=elapsed,
                method="opencv_cascade"
            )
        
        return DetectionResult(False, 0, 0, None, None, elapsed, "opencv_cascade")
    
    def detect_opencv_dnn(self, bgr_image: np.ndarray) -> DetectionResult:
        """Detect face using OpenCV DNN (ResNet-SSD)"""
        if self.opencv_dnn is None:
            return DetectionResult(False, 0, 0, None, None, 0, "opencv_dnn")
        
        start = time.perf_counter()
        h, w = bgr_image.shape[:2]
        
        blob = cv2.dnn.blobFromImage(
            bgr_image, 1.0, (300, 300), (104.0, 177.0, 123.0), False, False
        )
        self.opencv_dnn.setInput(blob)
        detections = self.opencv_dnn.forward()
        elapsed = (time.perf_counter() - start) * 1000
        
        # Find best detection
        best_conf = 0
        best_bbox = None
        
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5 and confidence > best_conf:
                best_conf = confidence
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = box.astype(int)
                best_bbox = (x1, y1, x2-x1, y2-y1)
        
        if best_bbox:
            x, y, bw, bh = best_bbox
            nose_pos = (x + bw/2, y + bh*0.6)
            
            return DetectionResult(
                detected=True,
                confidence=best_conf,
                landmarks_count=0,
                nose_position=nose_pos,
                bbox=best_bbox,
                processing_time_ms=elapsed,
                method="opencv_dnn"
            )
        
        return DetectionResult(False, 0, 0, None, None, elapsed, "opencv_dnn")
    
    # =========================================================================
    # Test Runner
    # =========================================================================
    
    def test_single_image(self, ir_gray: np.ndarray, show_results: bool = True) -> dict:
        """Test all conversion methods and detectors on a single IR image"""
        
        conversion_methods = {
            "simple": self.convert_simple,
            "clahe": self.convert_clahe,
            "histogram_eq": self.convert_histogram_eq,
            "adaptive": self.convert_adaptive_threshold,
            "gamma_1.2": lambda img: self.convert_gamma_correction(img, 1.2),
            "gamma_1.5": lambda img: self.convert_gamma_correction(img, 1.5),
        }
        
        detection_methods = {
            "mediapipe_mesh": self.detect_mediapipe_mesh,
            "mediapipe_face": self.detect_mediapipe_face,
            "dlib": self.detect_dlib,
            "opencv_cascade": self.detect_opencv_cascade,
            "opencv_dnn": self.detect_opencv_dnn,
        }
        
        results = {}
        
        print("\n" + "="*70)
        print("IR FACE DETECTION TEST RESULTS")
        print("="*70)
        print(f"Image size: {ir_gray.shape}")
        print(f"Mean intensity: {np.mean(ir_gray):.1f}")
        print(f"Std deviation: {np.std(ir_gray):.1f}")
        print("-"*70)
        
        for conv_name, conv_func in conversion_methods.items():
            bgr = conv_func(ir_gray)
            results[conv_name] = {}
            
            print(f"\n📷 Conversion: {conv_name}")
            print("-"*40)
            
            for det_name, det_func in detection_methods.items():
                result = det_func(bgr)
                results[conv_name][det_name] = result
                
                status = "✅" if result.detected else "❌"
                print(f"  {status} {det_name:20s} | "
                      f"conf={result.confidence:.2f} | "
                      f"landmarks={result.landmarks_count:3d} | "
                      f"time={result.processing_time_ms:6.1f}ms")
                
                if result.detected and result.nose_position:
                    print(f"      └─ nose: ({result.nose_position[0]:.1f}, {result.nose_position[1]:.1f})")
        
        # Summary
        print("\n" + "="*70)
        print("SUMMARY: Detection Success Rate by Conversion Method")
        print("="*70)
        
        for conv_name in conversion_methods.keys():
            successes = sum(1 for r in results[conv_name].values() if r.detected)
            total = len(detection_methods)
            rate = successes / total * 100
            bar = "█" * int(rate/10) + "░" * (10 - int(rate/10))
            print(f"  {conv_name:15s}: {bar} {rate:5.1f}% ({successes}/{total})")
        
        # Best conversion method
        best_conv = max(conversion_methods.keys(), 
                       key=lambda c: sum(1 for r in results[c].values() if r.detected))
        print(f"\n🏆 Best conversion method: {best_conv}")
        
        # Visualization
        if show_results:
            self._visualize_results(ir_gray, results, conversion_methods)
        
        return results
    
    def _visualize_results(self, ir_gray: np.ndarray, results: dict, 
                          conversion_methods: dict):
        """Create visualization of detection results"""
        
        # Create a grid of converted images with detections
        n_cols = 3
        n_rows = 2
        cell_h, cell_w = 300, 300
        
        canvas = np.zeros((n_rows * cell_h, n_cols * cell_w, 3), dtype=np.uint8)
        
        for idx, (conv_name, conv_func) in enumerate(list(conversion_methods.items())[:6]):
            row = idx // n_cols
            col = idx % n_cols
            
            bgr = conv_func(ir_gray)
            bgr_resized = cv2.resize(bgr, (cell_w, cell_h))
            
            # Draw detections
            for det_name, result in results[conv_name].items():
                if result.detected and result.bbox:
                    # Scale bbox to cell size
                    scale_x = cell_w / ir_gray.shape[1]
                    scale_y = cell_h / ir_gray.shape[0]
                    x, y, w, h = result.bbox
                    x, y, w, h = int(x*scale_x), int(y*scale_y), int(w*scale_x), int(h*scale_y)
                    
                    color = (0, 255, 0) if "mediapipe" in det_name else (255, 255, 0)
                    cv2.rectangle(bgr_resized, (x, y), (x+w, y+h), color, 2)
                    
                    if result.nose_position:
                        nx = int(result.nose_position[0] * scale_x)
                        ny = int(result.nose_position[1] * scale_y)
                        cv2.circle(bgr_resized, (nx, ny), 5, (0, 0, 255), -1)
            
            # Add label
            cv2.putText(bgr_resized, conv_name, (10, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Count successes
            successes = sum(1 for r in results[conv_name].values() if r.detected)
            cv2.putText(bgr_resized, f"{successes}/5 detected", (10, cell_h - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
            # Place in canvas
            y_start = row * cell_h
            x_start = col * cell_w
            canvas[y_start:y_start+cell_h, x_start:x_start+cell_w] = bgr_resized
        
        cv2.imshow("IR Face Detection Test Results", canvas)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def run_live_test(self, duration_seconds: int = 30):
        """Run live test with RealSense IR stream"""
        if not REALSENSE_AVAILABLE:
            print("❌ pyrealsense2 not available for live test")
            return
        
        print("\n" + "="*70)
        print("LIVE IR FACE DETECTION TEST")
        print("="*70)
        print("Press 'q' to quit, 's' to save screenshot")
        print("-"*70)
        
        # Configure RealSense
        pipeline = rs.pipeline()
        config = rs.config()
        
        # Enable only IR stream (left camera)
        config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 30)
        
        # Disable laser for pure passive IR
        pipeline.start(config)
        device = pipeline.get_active_profile().get_device()
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
            print("✅ Laser emitter disabled (passive IR mode)")
        
        # Stats tracking
        stats = {
            "frames": 0,
            "detections": 0,
            "total_time_ms": 0,
        }
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration_seconds:
                frames = pipeline.wait_for_frames()
                ir_frame = frames.get_infrared_frame(1)
                
                if not ir_frame:
                    continue
                
                # Convert to numpy
                ir_gray = np.asanyarray(ir_frame.get_data())
                
                # Use CLAHE conversion (best balance)
                bgr = self.convert_clahe(ir_gray)
                
                # Detect with MediaPipe
                result = self.detect_mediapipe_mesh(bgr)
                
                stats["frames"] += 1
                stats["total_time_ms"] += result.processing_time_ms
                if result.detected:
                    stats["detections"] += 1
                
                # Visualization
                display = bgr.copy()
                
                if result.detected:
                    if result.bbox:
                        x, y, w, h = result.bbox
                        cv2.rectangle(display, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    if result.nose_position:
                        nx, ny = int(result.nose_position[0]), int(result.nose_position[1])
                        cv2.circle(display, (nx, ny), 8, (0, 0, 255), -1)
                        cv2.putText(display, f"Nose: ({nx}, {ny})", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    cv2.putText(display, f"DETECTED ({result.landmarks_count} landmarks)", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(display, "NO FACE", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Stats overlay
                fps = stats["frames"] / (time.time() - start_time)
                det_rate = stats["detections"] / stats["frames"] * 100 if stats["frames"] > 0 else 0
                avg_time = stats["total_time_ms"] / stats["frames"] if stats["frames"] > 0 else 0
                
                cv2.putText(display, f"FPS: {fps:.1f} | Det: {det_rate:.0f}% | Avg: {avg_time:.1f}ms",
                           (10, display.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow("IR Face Detection - Live Test", display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"ir_capture_{int(time.time())}.png"
                    cv2.imwrite(filename, ir_gray)
                    print(f"📸 Saved: {filename}")
        
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
        
        # Print final stats
        print("\n" + "="*70)
        print("LIVE TEST RESULTS")
        print("="*70)
        print(f"Total frames:     {stats['frames']}")
        print(f"Detections:       {stats['detections']}")
        print(f"Detection rate:   {stats['detections']/stats['frames']*100:.1f}%")
        print(f"Average time:     {stats['total_time_ms']/stats['frames']:.1f}ms")
        print(f"Average FPS:      {stats['frames']/(time.time()-start_time):.1f}")
    
    def run_benchmark(self, num_frames: int = 100):
        """Run comprehensive benchmark with RealSense"""
        if not REALSENSE_AVAILABLE:
            print("❌ pyrealsense2 not available for benchmark")
            return
        
        print("\n" + "="*70)
        print(f"BENCHMARK: {num_frames} frames")
        print("="*70)
        
        # Configure RealSense
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 30)
        
        pipeline.start(config)
        device = pipeline.get_active_profile().get_device()
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 0)
        
        # Benchmark data
        conversion_methods = {
            "simple": self.convert_simple,
            "clahe": self.convert_clahe,
            "histogram_eq": self.convert_histogram_eq,
        }
        
        benchmark_results = {conv: {"detections": 0, "total_time": 0} 
                           for conv in conversion_methods}
        
        try:
            for i in range(num_frames):
                frames = pipeline.wait_for_frames()
                ir_frame = frames.get_infrared_frame(1)
                if not ir_frame:
                    continue
                
                ir_gray = np.asanyarray(ir_frame.get_data())
                
                for conv_name, conv_func in conversion_methods.items():
                    bgr = conv_func(ir_gray)
                    result = self.detect_mediapipe_mesh(bgr)
                    
                    benchmark_results[conv_name]["total_time"] += result.processing_time_ms
                    if result.detected:
                        benchmark_results[conv_name]["detections"] += 1
                
                if (i + 1) % 10 == 0:
                    print(f"  Progress: {i+1}/{num_frames}")
        
        finally:
            pipeline.stop()
        
        # Print results
        print("\n" + "="*70)
        print("BENCHMARK RESULTS")
        print("="*70)
        print(f"{'Conversion':<15} {'Detection %':>12} {'Avg Time (ms)':>15}")
        print("-"*45)
        
        for conv_name, data in benchmark_results.items():
            det_rate = data["detections"] / num_frames * 100
            avg_time = data["total_time"] / num_frames
            print(f"{conv_name:<15} {det_rate:>11.1f}% {avg_time:>14.1f}")
        
        # Recommendation
        best = max(benchmark_results.keys(), 
                  key=lambda k: benchmark_results[k]["detections"])
        print(f"\n🏆 Recommended conversion: {best}")


def main():
    parser = argparse.ArgumentParser(description="Test IR face detection effectiveness")
    parser.add_argument("--live", action="store_true", help="Run live test with RealSense")
    parser.add_argument("--benchmark", action="store_true", help="Run benchmark")
    parser.add_argument("--frames", type=int, default=100, help="Frames for benchmark")
    parser.add_argument("--image", type=str, help="Test with a saved IR image")
    parser.add_argument("--duration", type=int, default=30, help="Live test duration (seconds)")
    
    args = parser.parse_args()
    
    tester = IRFaceDetectionTester()
    
    if args.live:
        tester.run_live_test(args.duration)
    elif args.benchmark:
        tester.run_benchmark(args.frames)
    elif args.image:
        ir_gray = cv2.imread(args.image, cv2.IMREAD_GRAYSCALE)
        if ir_gray is None:
            print(f"❌ Failed to load image: {args.image}")
            return
        tester.test_single_image(ir_gray)
    else:
        # Demo with synthetic IR-like image
        print("No input specified. Creating synthetic test image...")
        
        # Try to capture one frame from RealSense
        if REALSENSE_AVAILABLE:
            try:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.infrared, 1, 848, 480, rs.format.y8, 30)
                pipeline.start(config)
                
                # Disable laser
                device = pipeline.get_active_profile().get_device()
                depth_sensor = device.first_depth_sensor()
                if depth_sensor.supports(rs.option.emitter_enabled):
                    depth_sensor.set_option(rs.option.emitter_enabled, 0)
                
                print("📷 Capturing IR frame from RealSense...")
                print("   Position your face in front of the camera...")
                time.sleep(2)  # Give user time to position
                
                frames = pipeline.wait_for_frames()
                ir_frame = frames.get_infrared_frame(1)
                
                if ir_frame:
                    ir_gray = np.asanyarray(ir_frame.get_data())
                    pipeline.stop()
                    
                    # Save for future testing
                    cv2.imwrite("test_ir_capture.png", ir_gray)
                    print("💾 Saved: test_ir_capture.png")
                    
                    tester.test_single_image(ir_gray)
                else:
                    pipeline.stop()
                    print("❌ Failed to capture IR frame")
            except Exception as e:
                print(f"❌ RealSense error: {e}")
        else:
            print("Run with --image <path> to test a saved IR image")
            print("Or with --live to test with RealSense camera")


if __name__ == "__main__":
    main()

