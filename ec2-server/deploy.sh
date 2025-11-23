#!/bin/bash
set -e

PEM_KEY="$HOME/.ssh/mdaiws.pem"
EC2_HOST="ubuntu@mdai.mercle.ai"
EC2_PATH="/opt/mdai"

echo "🚀 Deploying to EC2 server..."

# Copy files
echo "📦 Copying files..."
scp -i "$PEM_KEY" server.py "$EC2_HOST:$EC2_PATH/"
scp -i "$PEM_KEY" mobile_session_portal.html "$EC2_HOST:$EC2_PATH/"
scp -i "$PEM_KEY" user_portal.html "$EC2_HOST:$EC2_PATH/"
scp -i "$PEM_KEY" requirements.txt "$EC2_HOST:$EC2_PATH/"

# Restart service
echo "🔄 Restarting service..."
ssh -i "$PEM_KEY" "$EC2_HOST" "cd $EC2_PATH && sudo systemctl restart mdai-server"

echo "✅ Deployment complete!"
echo "🌐 Check status: https://mdai.mercle.ai/"



