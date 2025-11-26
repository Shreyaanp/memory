#!/usr/bin/env python3
"""
Generate calibration data for BPU model quantization.
Uses face images or generates synthetic data if no images available.
"""

import numpy as np
import cv2
import os
import sys

def generate_calibration_data(output_dir, num_samples=100, image_size=(192, 192)):
    """Generate calibration images for quantization"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"Generating {num_samples} calibration samples ({image_size[0]}x{image_size[1]})...")
    
    for i in range(num_samples):
        # Generate synthetic face-like patterns
        # These help the quantization algorithm understand the data distribution
        
        img = np.zeros((image_size[0], image_size[1], 3), dtype=np.float32)
        
        # Add skin-tone background
        skin_tones = [
            (0.9, 0.75, 0.65),   # Light
            (0.8, 0.65, 0.55),   # Medium light
            (0.7, 0.55, 0.45),   # Medium
            (0.5, 0.35, 0.25),   # Medium dark
            (0.3, 0.2, 0.15),    # Dark
        ]
        skin = skin_tones[i % len(skin_tones)]
        
        # Create oval face shape
        center = (image_size[0] // 2, image_size[1] // 2)
        axes = (image_size[0] // 3, image_size[1] // 2 - 20)
        
        # Fill background
        img[:, :] = [0.2, 0.2, 0.25]  # Neutral background
        
        # Draw face oval
        cv2.ellipse(img, center, axes, 0, 0, 360, skin, -1)
        
        # Add some noise for variation
        noise = np.random.randn(*img.shape) * 0.05
        img = np.clip(img + noise, 0, 1)
        
        # Add random variations (brightness, contrast)
        brightness = np.random.uniform(0.8, 1.2)
        contrast = np.random.uniform(0.9, 1.1)
        img = np.clip((img - 0.5) * contrast + 0.5 * brightness, 0, 1)
        
        # Save as binary file (float32 NHWC format)
        output_path = os.path.join(output_dir, f"calibration_{i:04d}.bin")
        img.astype(np.float32).tofile(output_path)
        
        # Also save a preview image
        if i < 5:
            preview_path = os.path.join(output_dir, f"preview_{i:04d}.jpg")
            cv2.imwrite(preview_path, (img * 255).astype(np.uint8)[:, :, ::-1])
        
        if (i + 1) % 20 == 0:
            print(f"  Generated {i + 1}/{num_samples} samples")
    
    print(f"✅ Calibration data saved to: {output_dir}")
    print(f"   Total files: {num_samples}")
    print(f"   Format: float32 binary, shape (192, 192, 3)")

def generate_from_images(source_dir, output_dir, num_samples=100, image_size=(192, 192)):
    """Generate calibration data from existing face images"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Find all images
    image_exts = ('.jpg', '.jpeg', '.png', '.bmp')
    images = []
    for f in os.listdir(source_dir):
        if f.lower().endswith(image_exts):
            images.append(os.path.join(source_dir, f))
    
    if not images:
        print(f"No images found in {source_dir}")
        return False
    
    print(f"Found {len(images)} source images")
    print(f"Generating {num_samples} calibration samples...")
    
    for i in range(num_samples):
        # Pick a random image
        img_path = images[i % len(images)]
        img = cv2.imread(img_path)
        
        if img is None:
            continue
        
        # Convert BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Resize to model input size
        img = cv2.resize(img, image_size)
        
        # Normalize to [0, 1]
        img = img.astype(np.float32) / 255.0
        
        # Random augmentation
        if np.random.random() > 0.5:
            img = cv2.flip(img, 1)  # Horizontal flip
        
        # Save as binary
        output_path = os.path.join(output_dir, f"calibration_{i:04d}.bin")
        img.astype(np.float32).tofile(output_path)
        
        if (i + 1) % 20 == 0:
            print(f"  Processed {i + 1}/{num_samples} samples")
    
    print(f"✅ Calibration data saved to: {output_dir}")
    return True

if __name__ == "__main__":
    output_dir = "./calibration_data"
    num_samples = 100
    
    if len(sys.argv) > 1:
        # Use provided image directory
        source_dir = sys.argv[1]
        if os.path.isdir(source_dir):
            generate_from_images(source_dir, output_dir, num_samples)
        else:
            print(f"Directory not found: {source_dir}")
            print("Generating synthetic data instead...")
            generate_calibration_data(output_dir, num_samples)
    else:
        # Generate synthetic data
        generate_calibration_data(output_dir, num_samples)
    
    print("\nNext step: Run the BPU conversion using Horizon Docker toolchain")

