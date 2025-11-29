#!/bin/bash
# MDAI Watchdog - Extra protection layer
# Monitors mdai-system and restarts if down

LOG="/home/mercleDev/mdai_logs/watchdog.log"
CHECK_INTERVAL=15
MAX_RESTART_ATTEMPTS=0  # 0 = unlimited

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG"
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log "=========================================="
log "MDAI Watchdog started"
log "=========================================="

restart_count=0

while true; do
    # Check if mdai-system service is active
    if ! systemctl is-active --quiet mdai-system; then
        log "⚠️  mdai-system is DOWN!"
        
        # Check if it's in failed state
        status=$(systemctl is-failed mdai-system 2>/dev/null)
        if [ "$status" = "failed" ]; then
            log "   Service in failed state, resetting..."
            systemctl reset-failed mdai-system
        fi
        
        log "   Attempting restart..."
        systemctl restart mdai-system
        
        sleep 5
        
        if systemctl is-active --quiet mdai-system; then
            log "✅ mdai-system restarted successfully"
            restart_count=$((restart_count + 1))
            log "   Total restarts by watchdog: $restart_count"
        else
            log "❌ Failed to restart mdai-system"
            log "   Will retry in $CHECK_INTERVAL seconds..."
        fi
    fi
    
    sleep $CHECK_INTERVAL
done



