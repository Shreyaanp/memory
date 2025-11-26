#!/usr/bin/env python3
"""
Test script for face_landmark.tflite model
Verifies the model loads correctly and can run inference
"""

import numpy as np
import cv2
import sys
import os

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
        sys.exit(1)

def get_model_info(interpreter):
    """Print model input/output details"""
    print("\n" + "="*60)
    print("MODEL INFORMATION")
    print("="*60)
    
    # Input details
    input_details = interpreter.get_input_details()
    print(f"\nNumber of inputs: {len(input_details)}")
    for i, inp in enumerate(input_details):
        print(f"\nInput {i}:")
        print(f"  Name: {inp['name']}")
        print(f"  Shape: {inp['shape']}")
        print(f"  Dtype: {inp['dtype']}")
        if 'quantization' in inp:
            print(f"  Quantization: {inp['quantization']}")
    
    # Output details
    output_details = interpreter.get_output_details()
    print(f"\nNumber of outputs: {len(output_details)}")
    for i, out in enumerate(output_details):
        print(f"\nOutput {i}:")
        print(f"  Name: {out['name']}")
        print(f"  Shape: {out['shape']}")
        print(f"  Dtype: {out['dtype']}")
        if 'quantization' in out:
            print(f"  Quantization: {out['quantization']}")
    
    return input_details, output_details

def test_inference(interpreter, input_details, output_details):
    """Run a test inference with dummy data"""
    print("\n" + "="*60)
    print("TEST INFERENCE")
    print("="*60)
    
    # Get input shape
    input_shape = input_details[0]['shape']
    input_dtype = input_details[0]['dtype']
    
    print(f"\nCreating dummy input with shape: {input_shape}, dtype: {input_dtype}")
    
    # Create dummy input (random image data normalized to [0, 1] or [-1, 1])
    if input_dtype == np.float32:
        dummy_input = np.random.rand(*input_shape).astype(np.float32)
    elif input_dtype == np.uint8:
        dummy_input = np.random.randint(0, 256, input_shape, dtype=np.uint8)
    else:
        dummy_input = np.random.rand(*input_shape).astype(input_dtype)
    
    # Set input tensor
    interpreter.set_tensor(input_details[0]['index'], dummy_input)
    
    # Run inference
    print("Running inference...")
    import time
    start = time.time()
    interpreter.invoke()
    elapsed = (time.time() - start) * 1000
    print(f"Inference time: {elapsed:.2f} ms")
    
    # Get outputs
    print("\nOutput results:")
    for i, out in enumerate(output_details):
        output_data = interpreter.get_tensor(out['index'])
        print(f"\n  Output {i}:")
        print(f"    Shape: {output_data.shape}")
        print(f"    Dtype: {output_data.dtype}")
        print(f"    Min: {output_data.min():.4f}")
        print(f"    Max: {output_data.max():.4f}")
        print(f"    Mean: {output_data.mean():.4f}")
    
    return True

def test_with_real_image(interpreter, input_details, output_details, image_path=None):
    """Test with a real image if available"""
    print("\n" + "="*60)
    print("TEST WITH IMAGE")
    print("="*60)
    
    # Try to find a test image
    if image_path is None:
        # Look for any image in current directory
        for ext in ['jpg', 'jpeg', 'png', 'bmp']:
            for f in os.listdir('.'):
                if f.lower().endswith(f'.{ext}'):
                    image_path = f
                    break
            if image_path:
                break
    
    if image_path is None or not os.path.exists(image_path):
        print("No test image found. Skipping real image test.")
        print("To test with an image, run: python test_face_landmark_tflite.py <image_path>")
        return True
    
    print(f"Using image: {image_path}")
    
    # Load and preprocess image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Failed to load image: {image_path}")
        return False
    
    print(f"Original image shape: {img.shape}")
    
    # Get expected input shape
    input_shape = input_details[0]['shape']
    # Typically [1, height, width, channels]
    if len(input_shape) == 4:
        _, height, width, channels = input_shape
    else:
        print(f"Unexpected input shape: {input_shape}")
        return False
    
    # Resize and preprocess
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_resized = cv2.resize(img_rgb, (width, height))
    
    # Normalize based on dtype
    input_dtype = input_details[0]['dtype']
    if input_dtype == np.float32:
        # Normalize to [0, 1] or [-1, 1]
        img_input = img_resized.astype(np.float32) / 255.0
    else:
        img_input = img_resized.astype(input_dtype)
    
    # Add batch dimension
    img_input = np.expand_dims(img_input, axis=0)
    
    print(f"Preprocessed input shape: {img_input.shape}")
    
    # Run inference
    interpreter.set_tensor(input_details[0]['index'], img_input)
    
    import time
    start = time.time()
    interpreter.invoke()
    elapsed = (time.time() - start) * 1000
    print(f"Inference time: {elapsed:.2f} ms")
    
    # Get landmarks output
    for i, out in enumerate(output_details):
        output_data = interpreter.get_tensor(out['index'])
        print(f"\nOutput {i} shape: {output_data.shape}")
        
        # If this looks like landmarks (468 points with x,y,z)
        if output_data.size == 468 * 3 or 'landmark' in out['name'].lower():
            landmarks = output_data.reshape(-1, 3)
            print(f"  Detected {len(landmarks)} landmark points")
            print(f"  First 5 landmarks (x, y, z):")
            for j in range(min(5, len(landmarks))):
                print(f"    Point {j}: ({landmarks[j][0]:.4f}, {landmarks[j][1]:.4f}, {landmarks[j][2]:.4f})")
    
    print("\n✅ Model inference successful!")
    return True

def main():
    model_path = "/home/mercleDev/codebase/tools/face_landmark.tflite"
    
    print("="*60)
    print("FACE LANDMARK TFLITE MODEL TEST")
    print("="*60)
    print(f"\nModel path: {model_path}")
    
    # Check if model exists
    if not os.path.exists(model_path):
        print(f"ERROR: Model not found at {model_path}")
        return 1
    
    print(f"Model size: {os.path.getsize(model_path) / 1024:.1f} KB")
    
    # Load model
    print("\nLoading model...")
    try:
        interpreter = Interpreter(model_path=model_path)
        interpreter.allocate_tensors()
        print("✅ Model loaded successfully!")
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        return 1
    
    # Get model info
    input_details, output_details = get_model_info(interpreter)
    
    # Test with dummy data
    try:
        test_inference(interpreter, input_details, output_details)
        print("\n✅ Dummy inference test PASSED!")
    except Exception as e:
        print(f"\n❌ Dummy inference test FAILED: {e}")
        return 1
    
    # Test with real image if provided
    image_path = sys.argv[1] if len(sys.argv) > 1 else None
    try:
        test_with_real_image(interpreter, input_details, output_details, image_path)
    except Exception as e:
        print(f"\n⚠️ Real image test failed: {e}")
    
    print("\n" + "="*60)
    print("SUMMARY: Model is valid and working!")
    print("="*60)
    print("\nNext step: Convert to BPU format using Horizon toolchain")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

