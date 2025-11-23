#!/usr/bin/env python3
"""
Device Registration Tool
Registers RDK device with EC2 server during deployment
"""

import sys
import json
import requests
import subprocess
from datetime import datetime

def get_mac_address():
    """Get MAC address from wlan0 or eth0"""
    try:
        with open('/sys/class/net/wlan0/address', 'r') as f:
            return f.read().strip()
    except:
        try:
            with open('/sys/class/net/eth0/address', 'r') as f:
                return f.read().strip()
        except:
            return "unknown"

def get_hostname():
    """Get system hostname"""
    try:
        with open('/etc/hostname', 'r') as f:
            return f.read().strip()
    except:
        return "unknown"

def generate_device_keys():
    """Generate RSA key pair using OpenSSL"""
    print("[1/4] Generating RSA key pair...")
    
    # Create directory
    subprocess.run(['mkdir', '-p', '/etc/mdai'], check=True)
    
    # Generate private key
    subprocess.run([
        'openssl', 'genrsa',
        '-out', '/etc/mdai/device_private.pem',
        '2048'
    ], check=True, capture_output=True)
    
    # Generate public key
    subprocess.run([
        'openssl', 'rsa',
        '-in', '/etc/mdai/device_private.pem',
        '-pubout',
        '-out', '/etc/mdai/device_public.pem'
    ], check=True, capture_output=True)
    
    # Set permissions
    subprocess.run(['chmod', '600', '/etc/mdai/device_private.pem'], check=True)
    subprocess.run(['chmod', '644', '/etc/mdai/device_public.pem'], check=True)
    
    print("✅ Keys generated")
    
    # Read public key
    with open('/etc/mdai/device_public.pem', 'r') as f:
        return f.read()

def register_with_server(server_url, device_id, hardware_id, public_key):
    """Register device with EC2 server"""
    print(f"[2/4] Registering with server: {server_url}")
    
    payload = {
        "device_id": device_id,
        "hardware_id": hardware_id,
        "hostname": get_hostname(),
        "public_key": public_key,
        "timestamp": datetime.now().isoformat()
    }
    
    try:
        response = requests.post(
            f"{server_url}/api/register-device",
            json=payload,
            timeout=10
        )
        
        if response.status_code == 200:
            print("✅ Device registered successfully")
            result = response.json()
            print(f"   Server response: {result.get('message', 'OK')}")
            return True
        else:
            print(f"❌ Registration failed: {response.status_code}")
            print(f"   Response: {response.text}")
            return False
            
    except requests.exceptions.RequestException as e:
        print(f"❌ Connection error: {e}")
        return False

def save_device_info(device_id, hardware_id):
    """Save device info locally"""
    print("[3/4] Saving device configuration...")
    
    config = {
        "device_id": device_id,
        "hardware_id": hardware_id,
        "hostname": get_hostname(),
        "registered_at": datetime.now().isoformat()
    }
    
    with open('/etc/mdai/device_config.json', 'w') as f:
        json.dump(config, f, indent=2)
    
    subprocess.run(['chmod', '600', '/etc/mdai/device_config.json'], check=True)
    
    print("✅ Configuration saved")

def main():
    if len(sys.argv) < 2:
        print("Usage: sudo python3 register_device.py <server_url>")
        print("Example: sudo python3 register_device.py https://mdai.mercle.ai")
        sys.exit(1)
    
    server_url = sys.argv[1].rstrip('/')
    
    print("=" * 60)
    print(" RDK Device Registration")
    print("=" * 60)
    print()
    
    # Get device info
    hardware_id = get_mac_address()
    device_id = "MDAI-" + hardware_id.replace(':', '')
    
    print(f"Device ID:    {device_id}")
    print(f"Hardware ID:  {hardware_id}")
    print(f"Hostname:     {get_hostname()}")
    print(f"Server:       {server_url}")
    print()
    
    # Check if already registered
    try:
        with open('/etc/mdai/device_config.json', 'r') as f:
            existing_config = json.load(f)
            print(f"⚠️  Device already registered on {existing_config.get('registered_at')}")
            print("   Re-registration will update server records.")
            print()
    except:
        pass
    
    # Generate or load keys
    try:
        with open('/etc/mdai/device_public.pem', 'r') as f:
            public_key = f.read()
            print("[1/4] Using existing RSA keys")
    except:
        public_key = generate_device_keys()
    
    # Register with server
    if register_with_server(server_url, device_id, hardware_id, public_key):
        # Save configuration
        save_device_info(device_id, hardware_id)
        
        print()
        print("[4/4] Creating registration marker...")
        with open('/etc/mdai/.registered', 'w') as f:
            f.write(datetime.now().isoformat())
        print("✅ Registration marker created")
        
        print()
        print("=" * 60)
        print("✅ REGISTRATION COMPLETE")
        print("=" * 60)
        print()
        print(f"Device {device_id} is now registered and ready for use.")
        print("You can now use the admin portal to provision WiFi.")
        print()
        return 0
    else:
        print()
        print("=" * 60)
        print("❌ REGISTRATION FAILED")
        print("=" * 60)
        print()
        print("Please check:")
        print("  1. Server is accessible")
        print("  2. Network connection is working")
        print("  3. Server URL is correct")
        print()
        return 1

if __name__ == "__main__":
    sys.exit(main())


