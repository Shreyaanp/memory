#!/usr/bin/env python3
"""
Device Label Generator
Creates device-specific QR code labels for physical stickers
"""

import hashlib
import hmac
import qrcode
import json
import sys
from pathlib import Path

# Master secret (must match firmware)
MASTER_SECRET = "mdai-master-secret-key-v1-change-in-production"

def derive_device_key(hardware_id: str, master_secret: str) -> str:
    """
    Derive device-specific key using HMAC-SHA256
    Same algorithm as C++ implementation
    """
    h = hmac.new(
        master_secret.encode('utf-8'),
        hardware_id.encode('utf-8'),
        hashlib.sha256
    )
    return h.hexdigest()

def generate_device_label(hardware_id: str, device_name: str = None):
    """
    Generate device label with QR code containing device key
    """
    # Derive key
    device_key = derive_device_key(hardware_id, MASTER_SECRET)
    
    # Create label data
    label_data = {
        "type": "device_label",
        "hardware_id": hardware_id,
        "device_key": device_key,
        "device_name": device_name or f"Device-{hardware_id[:8]}"
    }
    
    # Generate QR code
    qr = qrcode.QRCode(
        version=None,  # Auto size
        error_correction=qrcode.constants.ERROR_CORRECT_H,  # High error correction
        box_size=10,
        border=4,
    )
    
    qr.add_data(json.dumps(label_data))
    qr.make(fit=True)
    
    img = qr.make_image(fill_color="black", back_color="white")
    
    # Save QR code
    filename = f"device_label_{hardware_id.replace(':', '_')}.png"
    img.save(filename)
    
    print("=" * 60)
    print("DEVICE LABEL GENERATED")
    print("=" * 60)
    print(f"Hardware ID: {hardware_id}")
    print(f"Device Name: {label_data['device_name']}")
    print(f"Device Key:  {device_key}")
    print(f"\nQR Code saved: {filename}")
    print("\n📋 PRINT THIS AND STICK ON DEVICE")
    print("=" * 60)
    
    # Also save as text file
    txt_filename = f"device_label_{hardware_id.replace(':', '_')}.txt"
    with open(txt_filename, 'w') as f:
        f.write(f"MDAI Device Label\n")
        f.write(f"==================\n\n")
        f.write(f"Hardware ID: {hardware_id}\n")
        f.write(f"Device Name: {label_data['device_name']}\n")
        f.write(f"Device Key:  {device_key}\n\n")
        f.write(f"Keep this secure!\n")
    
    print(f"Text file saved: {txt_filename}\n")
    
    return label_data

def get_hardware_id_from_system():
    """Get MAC address from current system"""
    import subprocess
    try:
        # Try wlan0
        result = subprocess.run(
            ['cat', '/sys/class/net/wlan0/address'],
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except:
        pass
    
    try:
        # Try eth0
        result = subprocess.run(
            ['cat', '/sys/class/net/eth0/address'],
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            return result.stdout.strip()
    except:
        pass
    
    return None

if __name__ == "__main__":
    print("MDAI Device Label Generator")
    print("=" * 60)
    
    if len(sys.argv) > 1:
        # Hardware ID provided as argument
        hardware_id = sys.argv[1]
        device_name = sys.argv[2] if len(sys.argv) > 2 else None
    else:
        # Try to get from current system
        hardware_id = get_hardware_id_from_system()
        
        if not hardware_id:
            print("\n❌ Error: Could not detect hardware ID")
            print("\nUsage:")
            print(f"  {sys.argv[0]} <hardware_id> [device_name]")
            print("\nExample:")
            print(f"  {sys.argv[0]} b4:2f:03:31:9a:35 'Device-RDK-001'")
            sys.exit(1)
        
        print(f"\n✓ Detected Hardware ID: {hardware_id}")
        device_name = input("\nEnter device name (optional, press Enter to skip): ").strip()
        device_name = device_name if device_name else None
    
    generate_device_label(hardware_id, device_name)


