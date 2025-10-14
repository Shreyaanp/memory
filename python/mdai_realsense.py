"""
Python bindings for MDai RealSense system

This module provides Python access to the C++ RealSense capture system
through ctypes, allowing Python consumers to process frames from the ring buffer.
"""

import ctypes
import numpy as np
from typing import Optional, Dict, Any, Tuple
import os
import sys

# Load the shared library
def load_library():
    """Load the MDai RealSense shared library"""
    lib_paths = [
        "./build/libmdai_realsense.so",
        "../build/libmdai_realsense.so",
        "/usr/local/lib/libmdai_realsense.so",
    ]
    
    for path in lib_paths:
        if os.path.exists(path):
            return ctypes.CDLL(path)
    
    raise RuntimeError("Could not find libmdai_realsense.so")

# This will be set when module is imported
_lib = None

class FrameBoxMetadata(ctypes.Structure):
    """C structure for FrameBox metadata"""
    _fields_ = [
        ("anti_spoofing_processed", ctypes.c_bool),
        ("face_recognition_processed", ctypes.c_bool),
        ("batch_ml_processed", ctypes.c_bool),
        ("anti_spoofing_score", ctypes.c_float),
        ("is_valid_frame", ctypes.c_bool),
        ("detected_face_id", ctypes.c_char * 256),
        ("face_confidence", ctypes.c_float),
        ("emotion", ctypes.c_char * 64),
        ("estimated_age", ctypes.c_int),
    ]

class FrameData:
    """Python wrapper for frame data from RealSense using unified access system"""
    
    def __init__(self, sequence_id: int, depth: Optional[np.ndarray] = None, 
                 color: Optional[np.ndarray] = None, ir_left: Optional[np.ndarray] = None,
                 ir_right: Optional[np.ndarray] = None):
        self.sequence_id = sequence_id
        self.depth = depth  # numpy array (H, W) uint16
        self.color = color  # numpy array (H, W, 3) uint8 BGR
        self.ir_left = ir_left  # numpy array (H, W) uint8
        self.ir_right = ir_right  # numpy array (H, W) uint8
        self.metadata = {}
        self.depth_width = 0
        self.depth_height = 0
        self.color_width = 0
        self.color_height = 0
        self.depth_scale = 0.001
        
    def has_depth(self) -> bool:
        """Check if depth data is available"""
        return self.depth is not None and self.depth.size > 0
    
    def has_color(self) -> bool:
        """Check if color data is available"""
        return self.color is not None and self.color.size > 0
    
    def has_ir_left(self) -> bool:
        """Check if IR left data is available"""
        return self.ir_left is not None and self.ir_left.size > 0
    
    def has_ir_right(self) -> bool:
        """Check if IR right data is available"""
        return self.ir_right is not None and self.ir_right.size > 0
        
    def get_depth_at(self, x: int, y: int, depth_scale: float = None) -> float:
        """Get depth value at pixel (x, y) in meters"""
        if self.depth is None:
            return 0.0
        if depth_scale is None:
            depth_scale = self.depth_scale
        h, w = self.depth.shape
        if 0 <= y < h and 0 <= x < w:
            return float(self.depth[y, x]) * depth_scale
        return 0.0
    
    def get_center_depth(self, depth_scale: float = None) -> float:
        """Get depth at center of image"""
        if self.depth is None:
            return 0.0
        h, w = self.depth.shape
        return self.get_depth_at(w // 2, h // 2, depth_scale)
    
    def to_numpy_dict(self) -> Dict[str, np.ndarray]:
        """Convert frame data to dictionary of numpy arrays"""
        result = {}
        if self.has_depth():
            result['depth'] = self.depth
        if self.has_color():
            result['color'] = self.color
        if self.has_ir_left():
            result['ir_left'] = self.ir_left
        if self.has_ir_right():
            result['ir_right'] = self.ir_right
        return result
    
    def to_json(self) -> Dict[str, Any]:
        """Convert frame metadata to JSON-serializable dictionary"""
        return {
            'sequence_id': self.sequence_id,
            'depth_width': self.depth_width,
            'depth_height': self.depth_height,
            'color_width': self.color_width,
            'color_height': self.color_height,
            'depth_scale': self.depth_scale,
            'has_depth': self.has_depth(),
            'has_color': self.has_color(),
            'has_ir_left': self.has_ir_left(),
            'has_ir_right': self.has_ir_right(),
            'metadata': self.metadata
        }


class RingBufferConsumer:
    """
    Python consumer for the C++ ring buffer
    
    This class allows Python code to read frames from the shared ring buffer
    by accessing shared memory or through C API calls.
    """
    
    def __init__(self, mode: str = "realtime"):
        """
        Initialize consumer
        
        Args:
            mode: "realtime" or "sequential"
        """
        self.mode = mode
        self.last_sequence_id = 0
        self._lib = _lib  # Reference to loaded library
        
    def get_latest_frame(self) -> Optional[FrameData]:
        """
        Get the latest frame from ring buffer (realtime mode)
        
        Returns:
            FrameData object or None if no frame available
        """
        # In a real implementation, this would call C++ API
        # For now, this is a placeholder showing the interface
        
        # TODO: Implement actual C++ bridge
        # Example of what it would do:
        # 1. Call C++ get_latest_frame() via ctypes
        # 2. Get pointer to frame data
        # 3. Copy data to numpy arrays
        # 4. Return FrameData object
        
        return None
    
    def get_next_frame(self) -> Optional[FrameData]:
        """
        Get next frame in sequence (sequential mode)
        
        Returns:
            FrameData object or None if no new frame
        """
        # Similar to get_latest_frame but for sequential access
        return None
    
    def release_frame(self, frame: FrameData):
        """
        Release a frame back to the ring buffer
        
        Args:
            frame: FrameData to release
        """
        pass


class PipelineStage:
    """Base class for Python pipeline stages"""
    
    def process(self, frame: FrameData) -> bool:
        """
        Process a frame
        
        Args:
            frame: FrameData to process
            
        Returns:
            True to continue pipeline, False to stop
        """
        raise NotImplementedError
    
    def get_name(self) -> str:
        """Get stage name"""
        return self.__class__.__name__
    
    def on_start(self):
        """Called when pipeline starts"""
        pass
    
    def on_stop(self):
        """Called when pipeline stops"""
        pass


class AntiSpoofingStage(PipelineStage):
    """
    Example anti-spoofing pipeline stage
    
    This demonstrates how to implement a Python consumer that
    performs anti-spoofing checks on frames.
    """
    
    def __init__(self, threshold: float = 0.5):
        self.threshold = threshold
        self.processed_count = 0
        self.valid_count = 0
    
    def process(self, frame: FrameData) -> bool:
        """
        Perform anti-spoofing check
        
        In a real implementation, this would:
        1. Extract depth and color data
        2. Run ML model for liveness detection
        3. Update frame metadata with score
        4. Return True/False based on threshold
        """
        self.processed_count += 1
        
        # Mock implementation: check depth at center
        if frame.depth is not None:
            center_depth = frame.get_center_depth()
            
            # Simple heuristic: valid if person is 0.5-2.0m away
            if 0.5 <= center_depth <= 2.0:
                score = 0.9
                is_valid = True
                self.valid_count += 1
            else:
                score = 0.3
                is_valid = False
            
            # Update metadata
            frame.metadata['anti_spoofing_score'] = score
            frame.metadata['is_valid_frame'] = is_valid
            frame.metadata['anti_spoofing_processed'] = True
            
            print(f"[AntiSpoofing] Frame {frame.sequence_id}: "
                  f"score={score:.2f}, valid={is_valid}, depth={center_depth:.2f}m")
            
            return is_valid
        
        return False
    
    def on_stop(self):
        print(f"[AntiSpoofing] Processed {self.processed_count} frames, "
              f"{self.valid_count} valid ({100*self.valid_count/max(1,self.processed_count):.1f}%)")


class FaceRecognitionStage(PipelineStage):
    """Example face recognition stage"""
    
    def __init__(self):
        self.processed_count = 0
    
    def process(self, frame: FrameData) -> bool:
        """
        Perform face recognition
        
        This stage only processes frames that passed anti-spoofing
        """
        # Check if anti-spoofing was done
        if not frame.metadata.get('anti_spoofing_processed', False):
            return False
        
        if not frame.metadata.get('is_valid_frame', False):
            return False
        
        self.processed_count += 1
        
        # Mock face recognition
        if frame.color is not None:
            # In real implementation: run face detection + recognition
            frame.metadata['detected_face_id'] = f"person_{frame.sequence_id % 10}"
            frame.metadata['face_confidence'] = 0.85
            
            print(f"[FaceRecognition] Frame {frame.sequence_id}: "
                  f"detected={frame.metadata['detected_face_id']}, "
                  f"confidence={frame.metadata['face_confidence']:.2f}")
            
            return True
        
        return False
    
    def on_stop(self):
        print(f"[FaceRecognition] Processed {self.processed_count} frames")


def create_mock_frame(sequence_id: int) -> FrameData:
    """
    Create a mock frame for testing
    
    This is for demonstration purposes when camera is not available
    """
    # Create mock depth data (640x480, 16-bit)
    depth = np.random.randint(500, 2000, (480, 640), dtype=np.uint16)
    
    # Create mock color data (640x480x3, 8-bit BGR)
    color = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    frame = FrameData(sequence_id, depth, color)
    return frame


# Example pipeline runner
class SimplePipeline:
    """Simple Python pipeline runner"""
    
    def __init__(self):
        self.stages = []
    
    def add_stage(self, stage: PipelineStage):
        """Add a stage to the pipeline"""
        self.stages.append(stage)
    
    def process_frame(self, frame: FrameData) -> bool:
        """Process frame through all stages"""
        for stage in self.stages:
            if not stage.process(frame):
                return False
        return True
    
    def start(self):
        """Start all stages"""
        for stage in self.stages:
            stage.on_start()
    
    def stop(self):
        """Stop all stages"""
        for stage in self.stages:
            stage.on_stop()


# Initialize library when module is imported
try:
    # _lib = load_library()
    # For now, library loading is optional
    _lib = None
except Exception as e:
    print(f"Warning: Could not load C++ library: {e}")
    print("Python-only mode will be used")
    _lib = None


if __name__ == "__main__":
    # Example usage
    print("=== MDai RealSense Python Bindings Example ===\n")
    
    # Create pipeline
    pipeline = SimplePipeline()
    pipeline.add_stage(AntiSpoofingStage(threshold=0.5))
    pipeline.add_stage(FaceRecognitionStage())
    
    pipeline.start()
    
    # Process mock frames
    print("Processing mock frames...\n")
    for i in range(10):
        frame = create_mock_frame(i)
        pipeline.process_frame(frame)
    
    pipeline.stop()
    
    print("\nDone!")

