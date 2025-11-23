# EC2 Server SSH Credentials

## 🔑 SSH Access Information

### Server Details
- **Domain:** `mdai.mercle.ai`
- **User:** `ubuntu`
- **SSH Key:** `~/.ssh/mdaiws.pem`
- **Key Location (Full Path):** `/home/mercleDev/.ssh/mdaiws.pem`
- **Key Permissions:** `400` (read-only for owner)

---

## 📡 Quick SSH Commands

### Connect to EC2 Server
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai
```

### Connect with Verbose Logging (for debugging)
```bash
ssh -v -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai
```

---

## 📁 File Transfer (SCP)

### Copy File TO EC2
```bash
# Single file
scp -i ~/.ssh/mdaiws.pem /local/file.txt ubuntu@mdai.mercle.ai:/opt/mdai/

# Directory (recursive)
scp -i ~/.ssh/mdaiws.pem -r /local/directory ubuntu@mdai.mercle.ai:/opt/mdai/
```

### Copy File FROM EC2
```bash
# Single file
scp -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai:/opt/mdai/server.py ./

# Directory (recursive)
scp -i ~/.ssh/mdaiws.pem -r ubuntu@mdai.mercle.ai:/opt/mdai/ ./ec2-backup/
```

---

## 🚀 Common EC2 Operations

### Deploy Updated Server Code
```bash
cd /home/mercleDev/codebase/ec2-server

# Method 1: Using deployment script
./deploy-to-ec2.sh

# Method 2: Manual deployment
scp -i ~/.ssh/mdaiws.pem server.py ubuntu@mdai.mercle.ai:/opt/mdai/
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo systemctl restart mdai-server'
```

### Check Server Status
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo systemctl status mdai-server'
```

### View Live Logs
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo journalctl -u mdai-server -f'
```

### Restart Server
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo systemctl restart mdai-server'
```

### Stop Server
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo systemctl stop mdai-server'
```

### Start Server
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo systemctl start mdai-server'
```

---

## 🗂️ Important EC2 Directories

### Server Application
```
/opt/mdai/
├── server.py          # Main FastAPI application
├── requirements.txt   # Python dependencies
└── venv/             # Python virtual environment
```

### Database
```
/var/lib/mdai/
└── mdai_server.db    # SQLite database
```

### Nginx Configuration
```
/etc/nginx/sites-available/mdai
/etc/nginx/sites-enabled/mdai
```

### System Service
```
/etc/systemd/system/mdai-server.service
```

### Logs
```
# Application logs
sudo journalctl -u mdai-server -n 100

# Nginx access logs
sudo tail -f /var/log/nginx/access.log

# Nginx error logs
sudo tail -f /var/log/nginx/error.log
```

---

## 🛠️ Troubleshooting

### SSH Connection Issues

#### Permission Denied (Publickey)
```bash
# Check key permissions
ls -la ~/.ssh/mdaiws.pem

# Fix permissions if needed
chmod 400 ~/.ssh/mdaiws.pem
```

#### Connection Timeout
```bash
# Check if server is reachable
ping mdai.mercle.ai

# Test connection on different port
nc -zv mdai.mercle.ai 22
```

#### SSH Host Key Changed
```bash
# Remove old host key (if IP changed)
ssh-keygen -R mdai.mercle.ai
```

### Server Not Responding

```bash
# Check if server process is running
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'ps aux | grep server.py'

# Check system resources
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'top -bn1 | head -20'

# Check disk space
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'df -h'

# Check memory usage
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'free -h'
```

---

## 🔐 Security Notes

### SSH Key Security
- ✅ Key has correct permissions (400)
- ✅ Key is stored in secure location (~/.ssh/)
- ⚠️ **NEVER** share this key publicly
- ⚠️ **NEVER** commit this key to git

### Backup SSH Key
```bash
# Create encrypted backup
cp ~/.ssh/mdaiws.pem ~/.ssh/mdaiws.pem.backup
gpg -c ~/.ssh/mdaiws.pem.backup

# Or copy to secure external location
cp ~/.ssh/mdaiws.pem /path/to/secure/backup/
```

---

## 📊 Monitoring Commands

### Check Server Health
```bash
curl http://mdai.mercle.ai/api/health
```

### Check Server Status
```bash
curl http://mdai.mercle.ai/api/status
```

### View Active Sessions
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sqlite3 /var/lib/mdai/mdai_server.db "SELECT * FROM mobile_sessions;"'
```

### Check Database Size
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'du -h /var/lib/mdai/mdai_server.db'
```

---

## 🚨 Emergency Commands

### Force Kill Server Process
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo pkill -9 -f server.py'
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'sudo systemctl start mdai-server'
```

### Clean Restart Everything
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai << 'EOF'
sudo systemctl stop mdai-server
sudo systemctl restart nginx
rm -f /var/lib/mdai/mdai_server.db
sudo systemctl start mdai-server
EOF
```

### Backup Database Before Major Changes
```bash
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai 'cp /var/lib/mdai/mdai_server.db /var/lib/mdai/mdai_server.db.backup'
```

---

## 🔄 Update Workflow

### Complete Server Update Process
```bash
# 1. Connect to server
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai

# 2. Backup current version
cd /opt/mdai
sudo cp server.py server.py.backup

# 3. Exit and upload new version
exit
scp -i ~/.ssh/mdaiws.pem /home/mercleDev/codebase/ec2-server/server.py ubuntu@mdai.mercle.ai:/tmp/

# 4. Move and restart
ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai << 'EOF'
sudo mv /tmp/server.py /opt/mdai/server.py
sudo chown ubuntu:ubuntu /opt/mdai/server.py
sudo systemctl restart mdai-server
sudo journalctl -u mdai-server -n 50
EOF
```

---

## 📝 Quick Reference Card

```
╔════════════════════════════════════════════════════════════╗
║              EC2 SERVER QUICK REFERENCE                    ║
╠════════════════════════════════════════════════════════════╣
║ SSH:     ssh -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai  ║
║ Deploy:  cd ec2-server && ./deploy-to-ec2.sh              ║
║ Logs:    journalctl -u mdai-server -f                     ║
║ Status:  systemctl status mdai-server                     ║
║ Restart: sudo systemctl restart mdai-server               ║
║ Health:  curl http://mdai.mercle.ai/api/health            ║
╚════════════════════════════════════════════════════════════╝
```

---

## 🌐 Server URLs

- **API Base:** `http://mdai.mercle.ai`
- **Health Check:** `http://mdai.mercle.ai/api/health`
- **Status:** `http://mdai.mercle.ai/api/status`
- **WebSocket (Mobile):** `ws://mdai.mercle.ai/ws/mobile`
- **WebSocket (Device):** `ws://mdai.mercle.ai/ws/device`

---

## 💾 Automated Backup Script

Create this script in your local machine:

```bash
#!/bin/bash
# File: ~/backup-ec2.sh

BACKUP_DIR="/home/mercleDev/ec2-backups/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

echo "Backing up EC2 server to: $BACKUP_DIR"

# Backup server code
scp -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai:/opt/mdai/server.py "$BACKUP_DIR/"

# Backup database
scp -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai:/var/lib/mdai/mdai_server.db "$BACKUP_DIR/"

# Backup nginx config
scp -i ~/.ssh/mdaiws.pem ubuntu@mdai.mercle.ai:/etc/nginx/sites-available/mdai "$BACKUP_DIR/nginx.conf"

echo "✅ Backup complete!"
ls -lh "$BACKUP_DIR"
```

Make it executable:
```bash
chmod +x ~/backup-ec2.sh
```

---

**Last Updated:** 2025-11-21
**Server Version:** v1.0-beta


