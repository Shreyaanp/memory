# mDai EC2 Server

FastAPI backend server for mDai liveness verification system.

## 📁 Files

**Core Files:**
- `server.py` - FastAPI backend server
- `mobile_session_portal.html` - Mobile app QR code generator portal
- `user_portal.html` - RDK device testing portal with live logs
- `requirements.txt` - Python dependencies

**Configuration:**
- `.env.production` - Production environment template (copy to `.env` on server)

**Deployment:**
- `deploy.sh` - Deploy to EC2 server

## 🚀 Quick Deploy

```bash
./deploy.sh
```

This will:
1. Copy updated files to EC2
2. Restart the mdai-server service
3. Changes go live at https://mdai.mercle.ai

## 🌐 Portals

- **Mobile Portal**: https://mdai.mercle.ai/ or https://mdai.mercle.ai/mobile
  - For mobile app users
  - Generate QR code for mobile app scanning
  
- **User Portal**: https://mdai.mercle.ai/user
  - For RDK device testing
  - Has START button and live WebSocket logs
  - Real-time progress monitoring

## 📊 Database Schema

SQLite database with tables:
- `devices` - RDK device registrations
- `mobile_sessions` - Verification session data
- `session_logs` - WebSocket event logs

## 🔧 Local Development

```bash
# Install dependencies
pip install -r requirements.txt

# Copy environment file
cp .env.production .env
# Edit .env with your values

# Run server
python server.py
```

## 🔐 Environment Variables

See `.env.production` for required variables:
- `SECRET_KEY` - JWT signing key
- `AES_KEY` - QR code encryption key
- `DATABASE_URL` - SQLite database path
- `ALLOWED_ORIGINS` - CORS origins
