#!/usr/bin/env python3
"""
WebSocket Middleware Test Client
Tests device and app connections with session storage
"""

import asyncio
import websockets
import json
import sys

MIDDLEWARE_URL = "ws://18.210.22.254:8080"
# Note: Change to your EC2 IP or use localhost for local testing

async def test_device_connection():
    """Test device WebSocket connection"""
    device_id = "DEVICE_001"
    uri = f"{MIDDLEWARE_URL}/ws/device?device_id={device_id}"
    
    print(f"🔌 Connecting as device: {device_id}")
    
    try:
        async with websockets.connect(uri) as ws:
            # Receive welcome message
            msg = await ws.recv()
            data = json.loads(msg)
            print(f"✅ Device connected: {data}")
            
            # Send warmed_up status
            await ws.send(json.dumps({
                "type": "status",
                "session_id": "test_session_123",
                "status": "warmed_up"
            }))
            print("📤 Sent: warmed_up status")
            
            # Wait a bit for app commands
            print("⏳ Waiting for app commands (10s)...")
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=10.0)
                data = json.loads(msg)
                print(f"📥 Received from app: {data}")
                
                # Respond with result
                if data.get('type') == 'start' or data.get('cmd') == 'start':
                    await ws.send(json.dumps({
                        "type": "result",
                        "session_id": "test_session_123",
                        "status": "success",
                        "pass_rate": 0.85
                    }))
                    print("📤 Sent: success result")
                    
            except asyncio.TimeoutError:
                print("⏰ No app commands received")
            
            print("✅ Device test complete")
            
    except Exception as e:
        print(f"❌ Device error: {e}")

async def test_app_connection():
    """Test app WebSocket connection"""
    session_id = "test_session_123"
    device_id = "DEVICE_001"
    uri = f"{MIDDLEWARE_URL}/ws/app?session_id={session_id}&target_device={device_id}"
    
    print(f"📱 Connecting as app: session={session_id}")
    
    try:
        async with websockets.connect(uri) as ws:
            # Receive welcome message
            msg = await ws.recv()
            data = json.loads(msg)
            print(f"✅ App connected: {data}")
            print(f"   Device online: {data.get('device_online')}")
            
            # Wait for device status
            print("⏳ Waiting for device status (5s)...")
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=5.0)
                data = json.loads(msg)
                print(f"📥 Received from device: {data}")
                
                # Send start command
                await ws.send(json.dumps({
                    "type": "command",
                    "cmd": "start"
                }))
                print("📤 Sent: start command")
                
                # Wait for result
                print("⏳ Waiting for result...")
                msg = await asyncio.wait_for(ws.recv(), timeout=15.0)
                data = json.loads(msg)
                print(f"📥 Final result: {data}")
                
            except asyncio.TimeoutError:
                print("⏰ Timeout waiting for messages")
            
            print("✅ App test complete")
            
    except Exception as e:
        print(f"❌ App error: {e}")

async def test_both():
    """Run both device and app simultaneously"""
    print("=" * 60)
    print("WebSocket Middleware Test")
    print("=" * 60)
    
    # Run device and app in parallel
    await asyncio.gather(
        test_device_connection(),
        asyncio.sleep(2),  # Delay app start
        test_app_connection()
    )
    
    print("\n" + "=" * 60)
    print("Test Complete - Check status endpoint for session data")
    print("=" * 60)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "device":
            asyncio.run(test_device_connection())
        elif sys.argv[1] == "app":
            asyncio.run(test_app_connection())
        else:
            print("Usage: test_websocket.py [device|app]")
    else:
        asyncio.run(test_both())


