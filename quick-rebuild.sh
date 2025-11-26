#!/bin/bash
# Quick rebuild and restart - minimal output
# Usage: ./quick-rebuild.sh

set -e

cd /home/mercleDev/codebase/build
cmake .. > /dev/null 2>&1
make -j$(nproc) > /dev/null 2>&1

sudo pkill -9 mdai_system 2>/dev/null || true
sleep 2

cd /home/mercleDev/codebase
nohup ./build/mdai_system > /tmp/mdai_live.log 2>&1 &
sleep 2

if pgrep -f mdai_system > /dev/null; then
    echo "✅ Rebuild and restart complete"
    echo "📋 Logs: tail -f /tmp/mdai_live.log"
else
    echo "❌ Failed to start - check /tmp/mdai_live.log"
    exit 1
fi

