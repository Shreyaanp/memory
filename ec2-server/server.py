#!/usr/bin/env python3
"""
MDAI WebSocket Middleware Server - FastAPI Implementation
==========================================================
Handles:
- Mobile session creation with bearer token validation
- Encrypted QR generation (AES-256 with zlib compression)
- WebSocket pairing (mobile ↔ device)
- Message routing with platform_id privacy
- Multi-device scan detection
"""

import asyncio
import httpx
import json
import sqlite3
import secrets
import hashlib
import hmac
import time
import os
import zlib
from datetime import datetime, timedelta
from typing import Dict, Optional
import logging
from pathlib import Path
import base64
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend
from bson import ObjectId
import jwt as pyjwt
from dotenv import load_dotenv

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Header, Request
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import uvicorn

# Load .env file
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ============================================================================
# Configuration
# ============================================================================
SERVER_HOST = '0.0.0.0'
SERVER_PORT = 8000
DB_PATH = os.getenv('DB_PATH', './mdai_server.db')

# JWT Configuration (for WebSocket tokens)
JWT_SECRET = os.getenv('JWT_SECRET', 'mdai_secret_change_in_production_2024')
JWT_ALGORITHM = 'HS256'
JWT_EXPIRY = 5 * 60  # 5 minutes (for QR code scanning window)

# AES-256 Key for QR encryption
# QR Encryption Key - should be 64 hex chars in .env (will be converted to 32 bytes in aes_encrypt)
QR_ENCRYPTION_KEY = os.getenv('QR_ENCRYPTION_KEY', 'default_fallback_key_should_not_be_used_in_production_environment')

# Your Backend API Configuration
YOUR_BACKEND_VALIDATE_URL = os.getenv('BACKEND_VALIDATE_URL', 'https://newapi.mercle.ai/api/auth/verify-mdai-user')
YOUR_BACKEND_API_URL = 'https://newapi.mercle.ai/api/admin/hardware/next'
YOUR_BACKEND_API_KEY = os.getenv('BACKEND_API_KEY', 'mercle')

# Session expiry times
MOBILE_SESSION_EXPIRY = 5 * 60  # 5 minutes (match JWT expiry)

# ============================================================================
# FastAPI App
# ============================================================================
app = FastAPI(title="MDAI WebSocket Middleware", version="2.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# AES-256 Encryption/Decryption
# ============================================================================
def aes_encrypt(plaintext: str, key_hex: str) -> str:
    """Encrypt data with AES-256-CBC"""
    iv = secrets.token_bytes(16)
    pad_length = 16 - (len(plaintext) % 16)
    padded = plaintext + (chr(pad_length) * pad_length)
    
    # Convert hex key to bytes (not just encode!)
    key_bytes = bytes.fromhex(key_hex[:64])  # Take first 64 hex chars = 32 bytes
    
    cipher = Cipher(
        algorithms.AES(key_bytes),
        modes.CBC(iv),
        backend=default_backend()
    )
    encryptor = cipher.encryptor()
    ciphertext = encryptor.update(padded.encode()) + encryptor.finalize()
    
    return base64.b64encode(iv + ciphertext).decode()

# ============================================================================
# Database Schema
# ============================================================================
def init_database():
    """Initialize SQLite database"""
    db_dir = Path(DB_PATH).parent
    if db_dir != Path('.'):
        db_dir.mkdir(parents=True, exist_ok=True)
    
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    
    # Mobile sessions table
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS mobile_sessions (
            session_id TEXT PRIMARY KEY,
            platform_id TEXT NOT NULL,
            user_id TEXT,
            ws_token TEXT NOT NULL,
            backend_token TEXT,
            created_at INTEGER NOT NULL,
            expires_at TIMESTAMP NOT NULL,
            mobile_connected BOOLEAN DEFAULT 0,
            device_connected BOOLEAN DEFAULT 0,
            device_id TEXT,
            status TEXT DEFAULT 'pending',
            scan_count INTEGER DEFAULT 0,
            multi_scan_detected BOOLEAN DEFAULT 0,
            qr_scanned BOOLEAN DEFAULT 0
        )
    ''')
    
    # Message log
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS messages (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id TEXT,
            device_id TEXT,
            platform_id TEXT,
            direction TEXT,
            message_type TEXT,
            payload TEXT,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    
    # Audit log
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS audit_log (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            session_id TEXT,
            event_type TEXT,
            device_id TEXT,
            platform_id TEXT,
            details TEXT,
            timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    ''')
    
    cursor.execute('CREATE INDEX IF NOT EXISTS idx_mobile_sessions ON mobile_sessions(session_id)')
    try:
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_mobile_status ON mobile_sessions(status)')
    except sqlite3.OperationalError:
        # Column might not exist in old schema, skip
        pass
    
    conn.commit()
    conn.close()
    logger.info("Database initialized successfully")

# ============================================================================
# HMAC Token Helper Functions (Replacing JWT for smaller QR codes)
# ============================================================================
def generate_hmac_token(session_id: str, timestamp: int) -> str:
    """
    Generate HMAC token for WebSocket authentication
    Returns 8-char HMAC hash (64-bit security)
    QR will contain: just this token (no JSON, no timestamp)
    Server uses DB created_at for timestamp verification
    """
    # Create HMAC of session_id:timestamp
    message = f"{session_id}:{timestamp}".encode('utf-8')
    hmac_value = hmac.new(JWT_SECRET.encode(), message, hashlib.sha256).hexdigest()
    
    # Return first 8 chars (64 bits of security)
    return hmac_value[:8]

def verify_hmac_token(token: str, session_id: str, created_at_timestamp: int, max_age_seconds: int = 300) -> bool:
    """
    Verify HMAC token matches the expected value for given session_id
    Uses DB created_at timestamp (Unix timestamp integer)
    
    Args:
        token: HMAC hash (8 chars)
        session_id: Session ID from database lookup
        created_at_timestamp: Unix timestamp when session was created (from DB)
        max_age_seconds: Maximum token age (default 5 minutes)
    
    Returns:
        True if valid, False otherwise
    """
    try:
        # Check session not too old
        current_time = int(time.time())
        if current_time - created_at_timestamp > max_age_seconds:
            logger.warning(f"Token expired: {current_time - created_at_timestamp}s old")
            return False
        
        # Recompute HMAC using the original timestamp
        message = f"{session_id}:{created_at_timestamp}".encode('utf-8')
        hmac_expected = hmac.new(JWT_SECRET.encode(), message, hashlib.sha256).hexdigest()[:8]
        
        # Constant-time comparison
        return hmac.compare_digest(token, hmac_expected)
            
    except Exception as e:
        logger.warning(f"Token verification error: {e}")
        return False

# ============================================================================
# Session Pair Manager (Mobile ↔ Device)
# ============================================================================
class SessionPair:
    """Manages a paired mobile-device session"""
    def __init__(self, session_id: str, platform_id: str):
        self.session_id = session_id
        self.platform_id = platform_id
        self.mobile_ws: Optional[WebSocket] = None
        self.device_ws: Optional[WebSocket] = None
        self.device_id: Optional[str] = None
        self.created_at = datetime.now()
        
    async def route_to_device(self, message: dict):
        """Route message from mobile to device (strip platform_id)"""
        if not self.device_ws:
            return False
        
        # Preserve the 'type: to_device' wrapper and forward the data inside
        sanitized = {k: v for k, v in message.items() if k != 'platform_id'}
        
        try:
            await self.device_ws.send_json(sanitized)
            return True
        except Exception as e:
            logger.error(f"Error sending to device: {e}")
            return False
    
    async def route_to_mobile(self, message: dict):
        """Route message from device to mobile"""
        if not self.mobile_ws:
            return False
        
        try:
            await self.mobile_ws.send_json(message)
            return True
        except Exception as e:
            logger.error(f"Error sending to mobile: {e}")
            return False

# Active session pairs
active_sessions: Dict[str, SessionPair] = {}

# ============================================================================
# REST API Endpoints
# ============================================================================
@app.get("/")

@app.get("/user")
async def user_portal():
    """Serve user portal with START button and live logs"""
    return FileResponse("user_portal.html")


@app.get("/api/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "timestamp": datetime.now().isoformat()}

@app.get("/api/status")
async def get_status():
    """Get server status"""
    try:
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        cursor.execute('SELECT COUNT(*) FROM mobile_sessions WHERE status = "active"')
        active_sessions_count = cursor.fetchone()[0]
        
        cursor.execute('SELECT COUNT(*) FROM mobile_sessions WHERE multi_scan_detected = 1')
        multi_scan_count = cursor.fetchone()[0]
        
        conn.close()
        
        return {
            "status": "running",
            "active_sessions": active_sessions_count,
            "websocket_pairs": len(active_sessions),
            "multi_scan_detected": multi_scan_count,
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/mobile/create-session")
async def create_mobile_session(request: Request, authorization: Optional[str] = Header(None)):
    """
    Create mobile session after validating bearer token
    GET /api/mobile/create-session
    Headers: Authorization: Bearer <YOUR_BACKEND_TOKEN>
    """
    try:
        if not authorization or not authorization.startswith('Bearer '):
            raise HTTPException(status_code=401, detail="Missing or invalid Authorization header")
        
        bearer_token = authorization[7:]
        
        # Validate token with YOUR backend
        logger.info(f"Validating bearer token with backend: {YOUR_BACKEND_VALIDATE_URL}")
        
        async with httpx.AsyncClient() as client:
            try:
                resp = await client.get(
                    YOUR_BACKEND_VALIDATE_URL,
                    headers={'Authorization': f'Bearer {bearer_token}'},
                    timeout=10.0
                )
                
                if resp.status_code != 200:
                    raise HTTPException(status_code=401, detail="Token verification failed")
                
                validation_result = resp.json()
                
                if not validation_result.get('success'):
                    raise HTTPException(status_code=401, detail="Token verification failed")
                
                platform_id = validation_result.get('platform_id')
                user_id = validation_result.get('user_id', 'unknown')
                
                if not platform_id:
                    raise HTTPException(status_code=500, detail="Platform ID not returned from backend")
                
                logger.info(f"Token validated. Platform ID: {platform_id}")
                
            except httpx.TimeoutException:
                raise HTTPException(status_code=504, detail="Backend validation timeout")
            except httpx.RequestError as e:
                raise HTTPException(status_code=503, detail=f"Backend connection error: {str(e)}")
        
        # Generate session ID
        session_id = str(ObjectId())
        
        # Generate timestamp for HMAC
        timestamp = int(time.time())
        
        # Generate HMAC token
        ws_token = generate_hmac_token(session_id, timestamp)
        
        # Calculate expiry
        expires_at = datetime.now() + timedelta(seconds=MOBILE_SESSION_EXPIRY)
        
        # Store session in database (store timestamp used for HMAC)
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        logger.info(f"Inserting session into DB: {session_id}")
        cursor.execute('''
            INSERT INTO mobile_sessions 
            (session_id, platform_id, user_id, ws_token, backend_token, expires_at, status, created_at)
            VALUES (?, ?, ?, ?, ?, ?, 'pending', ?)
        ''', (session_id, platform_id, user_id, ws_token, bearer_token, expires_at, timestamp))
        
        conn.commit()
        logger.info(f"Session committed to DB: {session_id}")
        
        # Verify it was saved
        cursor.execute('SELECT session_id FROM mobile_sessions WHERE session_id = ?', (session_id,))
        if cursor.fetchone():
            logger.info(f"Session verified in DB: {session_id}")
        else:
            logger.error(f"Session NOT FOUND in DB after commit: {session_id}")
        
        conn.close()
        
        # Create QR payload - just the raw token with prefix
        # Add "S:" prefix to differentiate from WiFi QR
        qr_data = "S:" + ws_token  # Session QR marker + HMAC hash
        
        # Encrypt the token directly
        encrypted_qr = aes_encrypt(qr_data, QR_ENCRYPTION_KEY)
        
        logger.info(f"Mobile session created: {session_id}, platform_id: {platform_id}")
        logger.info(f"QR data size: {len(qr_data)} chars (S: prefix + 8-char token)")
        
        # Determine WebSocket URL based on request origin
        request_host = request.headers.get('host', 'mdai.mercle.ai')
        ws_protocol = 'wss' if request.url.scheme == 'https' else 'ws'
        ws_url = f"{ws_protocol}://{request_host}/ws/mobile"
        
        return {
            "success": True,
            "session_id": session_id,
            "ws_token": ws_token,
            "ws_url": ws_url,
            "expires_in": MOBILE_SESSION_EXPIRY,
            "qr_encrypted": encrypted_qr
        }
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Session creation error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ============================================================================
# WebSocket Endpoints
# ============================================================================
@app.websocket("/ws/mobile")
async def websocket_mobile(websocket: WebSocket):
    """Handle WebSocket connections from mobile app"""
    await websocket.accept()
    
    session_id = None
    platform_id = None
    session_pair = None
    
    try:
        # First message must be authentication
        data = await asyncio.wait_for(websocket.receive_json(), timeout=10.0)
        
        if data.get('type') != 'auth':
            await websocket.close(code=4400)
            return
        
        bearer_token = data.get('bearer_token')
        
        if not bearer_token:
            await websocket.send_json({'type': 'auth_failed', 'error': 'Missing token'})
            await websocket.close(code=4401)
            return
        
        # Lookup session by ws_token (HMAC hash)
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT session_id, platform_id, expires_at, status, multi_scan_detected, created_at
            FROM mobile_sessions
            WHERE ws_token = ?
        ''', (bearer_token,))
        
        row = cursor.fetchone()
        
        if not row:
            conn.close()
            await websocket.send_json({'type': 'auth_failed', 'error': 'Session not found'})
            await websocket.close(code=4404)
            return
        
        session_id, platform_id, expires_at, status, multi_scan_detected, created_at = row
        
        # Verify HMAC token matches (using DB created_at for timestamp)
        if not verify_hmac_token(bearer_token, session_id, created_at):
            conn.close()
            await websocket.send_json({'type': 'auth_failed', 'error': 'Invalid or expired token'})
            await websocket.close(code=4401)
            return
        
        # Check expiry
        expires_dt = datetime.fromisoformat(expires_at)
        if datetime.now() > expires_dt:
            conn.close()
            await websocket.send_json({'type': 'auth_failed', 'error': 'Session expired'})
            await websocket.close(code=4401)
            return
        
        # Update session
        cursor.execute('''
            UPDATE mobile_sessions
            SET mobile_connected = 1, status = 'mobile_connected'
            WHERE session_id = ?
        ''', (session_id,))
        
        conn.commit()
        conn.close()
        
        # Create session pair
        if session_id not in active_sessions:
            session_pair = SessionPair(session_id, platform_id)
            active_sessions[session_id] = session_pair
        else:
            session_pair = active_sessions[session_id]
        
        session_pair.mobile_ws = websocket
        
        await websocket.send_json({
            'type': 'auth_success',
            'session_id': session_id,
            'message': 'Mobile connected successfully'
        })
        
        logger.info(f"Mobile connected: session={session_id}, platform_id={platform_id}")
        
        # Handle messages
        while True:
            data = await websocket.receive_json()
            msg_type = data.get('type')
            
            if msg_type == 'to_device':
                # Forward the entire message (including type wrapper) to device
                sent = await session_pair.route_to_device(data)
                if not sent:
                    await websocket.send_json({'type': 'error', 'error': 'Device not connected'})
            
            elif msg_type == 'ping':
                await websocket.send_json({'type': 'pong'})
            
            else:
                await websocket.send_json({'type': 'ack', 'message': 'Message received'})
    
    except WebSocketDisconnect:
        logger.info(f"Mobile disconnected: session={session_id}")
    except asyncio.TimeoutError:
        logger.warning("Mobile auth timeout")
    except Exception as e:
        logger.error(f"Mobile WS error: {e}")
    finally:
        if session_pair:
            session_pair.mobile_ws = None
            
            if session_pair.device_ws:
                try:
                    await session_pair.device_ws.send_json({
                        'type': 'mobile_disconnected',
                        'message': 'Mobile app disconnected'
                    })
                except:
                    pass
            
            if not session_pair.device_ws and session_id in active_sessions:
                del active_sessions[session_id]

@app.websocket("/ws/device")
async def websocket_device(websocket: WebSocket):
    """Handle WebSocket connections from RDK devices"""
    await websocket.accept()
    
    session_id = None
    device_id = None
    session_pair = None
    platform_id = None
    
    try:
        # First message must be authentication
        data = await asyncio.wait_for(websocket.receive_json(), timeout=10.0)
        
        if data.get('type') != 'auth':
            await websocket.close(code=4400)
            return
        
        bearer_token = data.get('bearer_token')
        device_id = data.get('device_id')
        
        if not bearer_token or not device_id:
            await websocket.send_json({'type': 'auth_failed', 'error': 'Missing token or device_id'})
            await websocket.close(code=4401)
            return
        
        # Lookup session by ws_token (HMAC hash)
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT session_id, platform_id, expires_at, status, device_connected, device_id, qr_scanned, created_at
            FROM mobile_sessions
            WHERE ws_token = ?
        ''', (bearer_token,))
        
        row = cursor.fetchone()
        
        if not row:
            conn.close()
            await websocket.send_json({'type': 'auth_failed', 'error': 'Session not found'})
            await websocket.close(code=4404)
            return
        
        session_id, platform_id, expires_at, status, device_connected, existing_device_id, qr_scanned, created_at = row
        
        # Verify HMAC token matches (using DB created_at for timestamp)
        if not verify_hmac_token(bearer_token, session_id, created_at):
            conn.close()
            await websocket.send_json({'type': 'auth_failed', 'error': 'Invalid or expired token'})
            await websocket.close(code=4401)
            return
        
        # ONE-TIME QR ENFORCEMENT (Strict Security)
        if qr_scanned:
            conn.close()
            logger.warning(f"QR already used for session {session_id}")
            await websocket.send_json({
                'type': 'auth_failed',
                'error': 'QR code already used. Please generate a new session.'
            })
            await websocket.close(code=4403)
            return
        
        # Multi-scan detection
        if device_connected and existing_device_id and existing_device_id != device_id:
            cursor.execute('''
                UPDATE mobile_sessions
                SET multi_scan_detected = 1, status = 'multi_scan_blocked'
                WHERE session_id = ?
            ''', (session_id,))
            conn.commit()
            conn.close()
            
            await websocket.send_json({
                'type': 'auth_failed',
                'error': 'QR was scanned by multiple mDai devices. Please try again.'
            })
            await websocket.close(code=4409)
            
            # Notify mobile
            if session_id in active_sessions:
                session_pair = active_sessions[session_id]
                if session_pair.mobile_ws:
                    try:
                        await session_pair.mobile_ws.send_json({
                            'type': 'error',
                            'error': 'QR was scanned by multiple mDai devices. Please try again.'
                        })
                    except:
                        pass
            
            return
        
        # Check expiry
        expires_dt = datetime.fromisoformat(expires_at)
        if datetime.now() > expires_dt:
            conn.close()
            await websocket.send_json({'type': 'auth_failed', 'error': 'Session expired'})
            await websocket.close(code=4401)
            return
        
        # Update session - mark QR as scanned (one-time use)
        cursor.execute('''
            UPDATE mobile_sessions
            SET device_connected = 1, device_id = ?, status = 'active', qr_scanned = 1
            WHERE session_id = ?
        ''', (device_id, session_id))
        
        conn.commit()
        conn.close()
        
        logger.info(f"Device {device_id} authenticated, QR marked as used for session {session_id}")
        
        # Get session pair
        if session_id not in active_sessions:
            session_pair = SessionPair(session_id, platform_id)
            active_sessions[session_id] = session_pair
        else:
            session_pair = active_sessions[session_id]
        
        session_pair.device_ws = websocket
        session_pair.device_id = device_id
        
        await websocket.send_json({
            'type': 'auth_success',
            'session_id': session_id,
            'device_id': device_id,
            'message': 'Device connected successfully'
        })
        
        # Notify mobile
        if session_pair.mobile_ws:
            try:
                await session_pair.mobile_ws.send_json({
                    'type': 'device_connected',
                    'device_id': device_id,
                    'message': 'Device connected and ready'
                })
            except:
                pass
        
        logger.info(f"Device connected: session={session_id}, device_id={device_id}")
        
        # Handle messages
        while True:
            data = await websocket.receive_json()
            msg_type = data.get('type')
            
            if msg_type == 'to_mobile':
                sent = await session_pair.route_to_mobile(data.get('data', {}))
                if not sent:
                    await websocket.send_json({'type': 'error', 'error': 'Mobile not connected'})
            
            elif msg_type == 'submit_result':
                image_base64 = data.get('image')
                consecutive_passes = data.get('consecutive_passes', 0)
                
                logger.info(f"Result from {device_id}: {consecutive_passes} passes")
                
                if consecutive_passes >= 5:
                    try:
                        async with httpx.AsyncClient() as client:
                            resp = await client.post(
                                YOUR_BACKEND_API_URL,
                                headers={'x-api-key': YOUR_BACKEND_API_KEY, 'content-type': 'application/json'},
                                json={'platform_id': platform_id, 'image_base64': image_base64},
                                timeout=60.0
                            )
                            
                            api_result = resp.json()
                            
                            result_msg = {
                                'type': 'verification_result',
                                'status': 'success' if resp.status_code == 200 else 'failed',
                                'data': api_result
                            }
                            
                            if session_pair.mobile_ws:
                                await session_pair.mobile_ws.send_json({**result_msg, 'platform_id': platform_id})
                            
                            await websocket.send_json(result_msg)
                            
                    except Exception as e:
                        logger.error(f"Backend API error: {e}")
                        error_msg = {'type': 'verification_result', 'status': 'error', 'error': str(e)}
                        await websocket.send_json(error_msg)
                        if session_pair.mobile_ws:
                            await session_pair.mobile_ws.send_json(error_msg)
                else:
                    fail_msg = {'type': 'verification_result', 'status': 'failed', 'reason': 'Insufficient liveness frames'}
                    await websocket.send_json(fail_msg)
                    if session_pair.mobile_ws:
                        await session_pair.mobile_ws.send_json(fail_msg)
            
            elif msg_type == 'ping':
                await websocket.send_json({'type': 'pong'})
    
    except WebSocketDisconnect:
        logger.info(f"Device disconnected: session={session_id}, device_id={device_id}")
    except asyncio.TimeoutError:
        logger.warning("Device auth timeout")
    except Exception as e:
        logger.error(f"Device WS error: {e}")
    finally:
        if session_pair:
            session_pair.device_ws = None
            
            if session_pair.mobile_ws:
                try:
                    await session_pair.mobile_ws.send_json({
                        'type': 'device_disconnected',
                        'message': 'Device disconnected'
                    })
                except:
                    pass
            
            if not session_pair.mobile_ws and session_id in active_sessions:
                del active_sessions[session_id]

# ============================================================================
# Startup/Shutdown
# ============================================================================
@app.on_event("startup")
async def startup_event():
    init_database()
    logger.info("=" * 70)
    logger.info(f"MDAI WebSocket Middleware Server Starting")
    logger.info("=" * 70)
    logger.info(f"Backend Validation URL: {YOUR_BACKEND_VALIDATE_URL}")
    logger.info(f"Backend API URL: {YOUR_BACKEND_API_URL}")
    logger.info("=" * 70)

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Server shutting down")

# ============================================================================
# Main
# ============================================================================
if __name__ == '__main__':
    uvicorn.run(app, host=SERVER_HOST, port=SERVER_PORT, log_level="info")
