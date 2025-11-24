# MDAI WebSocket Middleware Server

Secure WebSocket middleware server for real-time communication between mobile applications and RDK face detection devices.

## Overview

This server acts as a secure bridge between:
- **Mobile App** (Flutter): User interface for liveness verification
- **RDK Device** (C++/ESP32): Hardware performing face detection and liveness checks
- **Backend API**: Platform for storing verification results

## Features

- ✅ **Secure WebSocket Communication** (WSS with SSL/TLS)
- ✅ **Token-based Authentication** (Two-tier: User token + Session token)
- ✅ **Encrypted QR Code** (AES-256-CBC encryption)
- ✅ **Session Management** (3-minute expiry, multi-scan detection)
- ✅ **Privacy Protection** (Device never knows platform_id)
- ✅ **Real-time Message Routing** (Bidirectional mobile ↔ device)
- ✅ **Backend Integration** (Automatic result forwarding)
- ✅ **SQLite Database** (Session tracking and persistence)

## Architecture

```
Mobile App → EC2 Server → Backend (token validation)
     ↕           ↕
  WebSocket   WebSocket
     ↕           ↕
   QR Code ← RDK Device
```

## Quick Start

### Prerequisites

- Python 3.10+
- Virtual environment
- SSL certificate (Let's Encrypt for production)
- Backend API access

### Installation

```bash
# Clone repository
git clone https://github.com/Shreyaanp/mdaiServer.git
cd mdaiServer

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Edit .env with your configuration
```

### Configuration

Create `.env` file with:

```env
# JWT Secret for WebSocket tokens
JWT_SECRET=your_secret_key_here

# AES-256 Key for QR encryption (32 bytes hex)
QR_ENCRYPTION_KEY=your_hex_key_here

# Backend API
BACKEND_VALIDATE_URL=https://your-backend.com/api/auth/verify-mdai-user
BACKEND_API_KEY=your_api_key

# Database
DB_PATH=/var/lib/mdai/mdai_server.db

# Server
SERVER_HOST=0.0.0.0
SERVER_PORT=8000
```

### Running Locally

```bash
# Activate virtual environment
source venv/bin/activate

# Run server
python3 server.py
```

Server will be available at `http://localhost:8000`

### Running in Production

#### Using Systemd (Recommended)

1. Create systemd service file `/etc/systemd/system/mdai-server.service`:

```ini
[Unit]
Description=MDAI WebSocket Middleware Server
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/opt/mdai
Environment="PATH=/opt/mdai/venv/bin"
ExecStart=/opt/mdai/venv/bin/python3 /opt/mdai/server.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

2. Enable and start service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable mdai-server
sudo systemctl start mdai-server
sudo systemctl status mdai-server
```

#### Nginx Configuration (For WSS)

```nginx
server {
    server_name mdai.example.com;
    
    location /ws/ {
        proxy_pass http://127.0.0.1:8000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_read_timeout 86400;
    }
    
    location /api/ {
        proxy_pass http://127.0.0.1:8000;
        proxy_set_header Host $host;
    }
    
    listen 443 ssl;
    ssl_certificate /etc/letsencrypt/live/mdai.example.com/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/mdai.example.com/privkey.pem;
}
```

## API Endpoints

### REST API

#### `GET /api/status`
Server status and statistics.

#### `GET /api/mobile/create-session`
Create a new verification session.

**Headers:**
```
Authorization: Bearer <user_token>
```

**Response:**
```json
{
  "success": true,
  "session_id": "6922e054...",
  "ws_token": "eyJhbGc...",
  "ws_url": "wss://mdai.example.com/ws/mobile",
  "qr_encrypted": "dIzrgkmMDJnD...",
  "expires_in": 180
}
```

### WebSocket Endpoints

#### `WSS /ws/mobile?session_id={id}`
Mobile app WebSocket connection.

#### `WSS /ws/device?session_id={id}`
RDK device WebSocket connection.

## Message Protocol

### Mobile → Server

**Authentication:**
```json
{
  "type": "auth",
  "bearer_token": "eyJhbGc...",
  "session_id": "6922e054..."
}
```

**Send to Device:**
```json
{
  "type": "to_device",
  "data": { "action": "start", "params": {} }
}
```

### Device → Server

**Authentication:**
```json
{
  "type": "auth",
  "bearer_token": "eyJhbGc...",
  "device_id": "RDK_001",
  "session_id": "6922e054..."
}
```

**Submit Result:**
```json
{
  "type": "submit_result",
  "image": "base64_image...",
  "consecutive_passes": 5,
  "confidence": 0.95,
  "liveness_score": 0.92
}
```

**Progress Update:**
```json
{
  "type": "progress",
  "value": 60
}
```

### Server → Mobile/Device

**Authentication Success:**
```json
{
  "type": "auth_success",
  "session_id": "6922e054...",
  "message": "Connected successfully"
}
```

**Verification Result:**
```json
{
  "type": "verification_result",
  "status": "success",
  "message": "Verification successful"
}
```

## QR Code Format

The QR code contains an encrypted JSON payload:

**Encrypted Content:**
```json
{
  "session_id": "6922e054...",
  "token": "eyJhbGc...",
  "timestamp": 1732389414
}
```

**Encryption:** AES-256-CBC with 16-byte IV

## Database Schema

### `sessions` Table

```sql
CREATE TABLE sessions (
    session_id TEXT PRIMARY KEY,
    platform_id TEXT,
    ws_token TEXT,
    mobile_connected INTEGER,
    device_id TEXT,
    device_connected INTEGER,
    created_at TIMESTAMP,
    expires_at TIMESTAMP
);
```

### `device_scans` Table

```sql
CREATE TABLE device_scans (
    device_id TEXT,
    session_id TEXT,
    scan_timestamp TIMESTAMP,
    PRIMARY KEY (device_id, session_id)
);
```

## Security Features

### Two-Tier Authentication

1. **User Bearer Token**
   - Long-lived token from backend
   - Used only for session creation
   - Validated against backend API

2. **WebSocket Session Token**
   - Short-lived (3 minutes)
   - Generated by EC2 server
   - Used for WebSocket authentication

### Privacy Protection

- Device **never** receives `platform_id`
- Server stores `platform_id` from backend validation
- Server injects `platform_id` when forwarding results

### Session Security

- Automatic expiry after 3 minutes
- Multi-scan detection (blocks device reuse)
- Unique session IDs per verification

## Testing

### Local Testing

```bash
# Run test server
python3 server.py

# In another terminal, run tests
python3 test_full_flow.py
```

### Test Scripts

- `test_full_flow.py` - Complete end-to-end simulation
- `simulate_mobile.py` - Mobile app simulator
- `simulate_rdk.py` - RDK device simulator

## Deployment

### To Production EC2

```bash
# SSH to EC2
ssh ubuntu@mdai.example.com

# Pull latest changes
cd /opt/mdai
git pull origin main

# Install dependencies (if needed)
source venv/bin/activate
pip install -r requirements.txt

# Restart service
sudo systemctl restart mdai-server

# Check status
sudo systemctl status mdai-server

# Monitor logs
sudo journalctl -u mdai-server -f
```

## Monitoring

### Check Server Status

```bash
# Service status
sudo systemctl status mdai-server

# View logs
sudo journalctl -u mdai-server -n 100

# Follow logs in real-time
sudo journalctl -u mdai-server -f
```

### Health Check

```bash
curl https://mdai.example.com/api/status
```

## Troubleshooting

### Server won't start

```bash
# Check logs
sudo journalctl -u mdai-server -n 50

# Check if port is in use
sudo netstat -tlnp | grep 8000

# Check permissions
ls -la /var/lib/mdai/
```

### WebSocket connection fails

1. Check nginx configuration
2. Verify SSL certificate is valid
3. Check firewall rules (port 443)
4. Verify `.env` configuration

### Database issues

```bash
# Check database file
ls -lh /var/lib/mdai/mdai_server.db

# Check database schema
sqlite3 /var/lib/mdai/mdai_server.db ".schema"

# Backup database
cp /var/lib/mdai/mdai_server.db /var/lib/mdai/mdai_server.db.backup
```

## Development

### Project Structure

```
mdaiServer/
├── server.py              # Main FastAPI server
├── requirements.txt       # Python dependencies
├── .env                   # Environment configuration (not in git)
├── .gitignore            # Git ignore rules
├── README.md             # This file
├── MESSAGE_PROTOCOL.md   # Detailed message specifications
├── tests/                # Test utilities
└── mobile_session_portal.html  # Test UI (optional)
```

### Adding New Features

1. Create feature branch
2. Implement changes
3. Test locally with `test_full_flow.py`
4. Commit and push
5. Deploy to EC2

## License

Proprietary - Mercle Inc.

## Support

For issues or questions:
- GitHub: https://github.com/Shreyaanp/mdaiServer
- Email: support@mercle.ai

## Version History

- **v2.0** - Added .env support, message protocol documentation
- **v1.0** - Initial production release
