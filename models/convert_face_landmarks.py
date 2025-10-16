#!/usr/bin/env python3
"""
Download and convert Face Alignment Network (FAN) to ONNX format
This creates a high-quality 68-landmark face detector
"""

import torch
import torch.nn as nn
import onnx
import numpy as np
from urllib import request
import os

print("=" * 60)
print("Face Alignment Network (FAN) → ONNX Converter")
print("=" * 60)

# Download pre-trained FAN model (2D-FAN-4, best quality)
model_url = "https://www.adrianbulat.com/downloads/python-fan/2DFAN4-cd938726ad.zip"
model_path = "2DFAN4-cd938726ad.zip"

print("\n1. Trying to use face-alignment library (easiest)...")
try:
    import face_alignment
    print("✓ face-alignment library found!")
    
    # Create FAN model
    print("\n2. Creating Face Alignment Network...")
    from face_alignment import LandmarksType
    fa = face_alignment.FaceAlignment(
        LandmarksType.TWO_D,
        flip_input=False,
        device='cpu'
    )
    
    print("✓ Model loaded successfully!")
    
    # Export to ONNX
    print("\n3. Exporting to ONNX format...")
    dummy_input = torch.randn(1, 3, 256, 256)
    
    output_path = "face_landmarks_68.onnx"
    
    torch.onnx.export(
        fa.face_alignment_net,
        dummy_input,
        output_path,
        export_params=True,
        opset_version=11,
        do_constant_folding=True,
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={
            'input': {0: 'batch_size'},
            'output': {0: 'batch_size'}
        }
    )
    
    print(f"✓ Model exported to: {output_path}")
    
    # Verify the model
    print("\n4. Verifying ONNX model...")
    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)
    print("✓ ONNX model is valid!")
    
    # Print model info
    file_size = os.path.getsize(output_path) / (1024 * 1024)
    print(f"\n✅ SUCCESS!")
    print(f"   Model: face_landmarks_68.onnx")
    print(f"   Size: {file_size:.2f} MB")
    print(f"   Landmarks: 68 points (iBUG format)")
    print(f"   Input: 256x256 RGB image")
    print(f"   Output: 68x2 coordinates")
    
except ImportError:
    print("✗ face-alignment library not found")
    print("\n📥 Installing face-alignment library...")
    os.system("pip3 install face-alignment -q")
    print("\n🔄 Please run this script again")
    exit(0)
    
except Exception as e:
    print(f"✗ Error: {e}")
    print("\n" + "="*60)
    print("ALTERNATIVE: Manual Model Download")
    print("="*60)
    print("\n📥 Download one of these models manually:")
    print("\n1. 2D-FAN-4 (Best quality, ~25MB):")
    print("   https://www.adrianbulat.com/downloads/python-fan/2DFAN4-cd938726ad.zip")
    print("\n2. HRNet (Good balance, ~20MB):")
    print("   Search: 'HRNet face landmarks onnx' on GitHub")
    print("\n3. MobileFaceNet (Fastest, ~4MB):")
    print("   Search: 'MobileFaceNet landmarks onnx' on GitHub")
    print("\nSave as: face_landmarks_68.onnx in this directory")
    exit(1)

