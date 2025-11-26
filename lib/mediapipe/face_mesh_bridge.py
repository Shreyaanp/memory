#!/usr/bin/env python3
"""
Pure Python MediaPipe Face Mesh Bridge

This module wraps MediaPipe and returns simple Python dicts/lists
that can be easily consumed by the C++ wrapper without protobuf issues.
"""

import mediapipe as mp
import numpy as np

class FaceMeshBridge:
    """
    Simple bridge class that wraps MediaPipe Face Mesh
    and returns plain Python data structures.
    """
    
    def __init__(self, max_faces=1, min_detection_confidence=0.5, min_tracking_confidence=0.5):
        """Initialize the Face Mesh detector."""
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=max_faces,
            refine_landmarks=True,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self._initialized = True
    
    def process(self, rgb_image):
        """
        Process an RGB image and return face landmarks as simple Python lists.
        
        Args:
            rgb_image: numpy array of shape (H, W, 3) in RGB format
            
        Returns:
            dict with 'landmarks' (list of dicts), 'bbox', 'confidence'
            or None if no face detected
        """
        if not self._initialized:
            return None
        
        try:
            results = self.face_mesh.process(rgb_image)
        except Exception as e:
            print(f"[FaceMeshBridge] Process error: {e}")
            return None
        
        # No face detected
        if not results.multi_face_landmarks:
            return None
        
        # Get first face
        face_landmarks = results.multi_face_landmarks[0]
        
        # Convert landmarks to simple list of dicts
        landmarks = []
        min_x, min_y = 1.0, 1.0
        max_x, max_y = 0.0, 0.0
        
        for lm in face_landmarks.landmark:
            x = float(lm.x)
            y = float(lm.y)
            z = float(lm.z)
            
            # Get visibility/presence if available
            vis = float(getattr(lm, 'visibility', 1.0) or 1.0)
            pres = float(getattr(lm, 'presence', 1.0) or 1.0)
            
            landmarks.append({
                'x': x,
                'y': y,
                'z': z,
                'visibility': vis,
                'presence': pres
            })
            
            # Track bounding box
            if x < min_x: min_x = x
            if x > max_x: max_x = x
            if y < min_y: min_y = y
            if y > max_y: max_y = y
        
        # Calculate average presence as confidence
        avg_presence = sum(lm['presence'] for lm in landmarks) / len(landmarks) if landmarks else 0.0
        
        return {
            'landmarks': landmarks,
            'count': len(landmarks),
            'bbox': {
                'x': min_x,
                'y': min_y,
                'width': max_x - min_x,
                'height': max_y - min_y
            },
            'confidence': avg_presence
        }
    
    def close(self):
        """Clean up resources."""
        if self._initialized:
            try:
                self.face_mesh.close()
            except:
                pass
            self._initialized = False


# Global instance for C++ to use
_bridge_instance = None

def initialize(max_faces=1, min_detection_confidence=0.5, min_tracking_confidence=0.5):
    """Initialize the global bridge instance."""
    global _bridge_instance
    try:
        _bridge_instance = FaceMeshBridge(max_faces, min_detection_confidence, min_tracking_confidence)
        return True
    except Exception as e:
        print(f"[FaceMeshBridge] Init error: {e}")
        return False

def process_image(rgb_image):
    """
    Process an image using the global bridge instance.
    
    Args:
        rgb_image: numpy array (H, W, 3) RGB format
        
    Returns:
        dict with landmarks or None
    """
    global _bridge_instance
    if _bridge_instance is None:
        if not initialize():
            return None
    return _bridge_instance.process(rgb_image)

def cleanup():
    """Clean up the global bridge instance."""
    global _bridge_instance
    if _bridge_instance:
        _bridge_instance.close()
        _bridge_instance = None

def is_initialized():
    """Check if bridge is initialized."""
    global _bridge_instance
    return _bridge_instance is not None and _bridge_instance._initialized


# Test function
if __name__ == "__main__":
    import cv2
    
    print("Testing FaceMeshBridge...")
    
    # Initialize
    if not initialize():
        print("Failed to initialize")
        exit(1)
    
    print("✓ Initialized")
    
    # Create test image
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Process
    result = process_image(img)
    
    if result:
        print(f"✓ Face detected with {result['count']} landmarks")
    else:
        print("✓ No face (expected for blank image)")
    
    # Cleanup
    cleanup()
    print("✓ Cleanup complete")
    print("\n✅ Bridge test passed!")



