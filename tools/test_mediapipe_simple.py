#!/usr/bin/env python3
"""
Simple MediaPipe test to verify installation and basic functionality
"""

import sys

def test_mediapipe():
    print("Testing MediaPipe installation...")
    
    try:
        import mediapipe as mp
        print(f"✓ MediaPipe imported successfully (version: {mp.__version__})")
    except ImportError as e:
        print(f"✗ Failed to import MediaPipe: {e}")
        print("  Install with: pip3 install mediapipe")
        return False
    
    try:
        from mediapipe.solutions import face_mesh
        print("✓ MediaPipe face_mesh module available")
    except ImportError as e:
        print(f"✗ Failed to import face_mesh: {e}")
        return False
    
    try:
        FaceMesh = face_mesh.FaceMesh
        print("✓ FaceMesh class available")
    except AttributeError as e:
        print(f"✗ FaceMesh class not found: {e}")
        return False
    
    try:
        # Try to create an instance
        face_mesh_instance = FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        print("✓ FaceMesh instance created successfully")
        face_mesh_instance.close()
        print("✓ FaceMesh cleanup successful")
    except Exception as e:
        print(f"✗ Failed to create FaceMesh instance: {e}")
        return False
    
    print("\n✅ All MediaPipe tests passed!")
    return True

if __name__ == "__main__":
    success = test_mediapipe()
    sys.exit(0 if success else 1)

