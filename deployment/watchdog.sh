#!/bin/bash
# MDAI Enhanced Watchdog
# - Monitors mdai-system service
# - Detects crash loops
# - Auto-restarts with backoff
# - Logs all events

LOG="/home/mercleDev/mdai_logs/watchdog.log"
CRASH_LOG="/home/mercleDev/mdai_logs/crash_details.log"
CHECK_INTERVAL=15
MAX_CRASHES_PER_HOUR=10
CRASH_WINDOW=3600  # 1 hour in seconds

# Track crashes
CRASH_TIMES=()

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG"
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1"
}

count_recent_crashes() {
    local now=$(date +%s)
    local count=0
    local new_times=()
    
    for crash_time in "${CRASH_TIMES[@]}"; do
        if (( now - crash_time < CRASH_WINDOW )); then
            new_times+=("$crash_time")
            ((count++))
        fi
    done
    
    CRASH_TIMES=("${new_times[@]}")
    echo $count
}

log "=========================================="
log "MDAI Enhanced Watchdog started"
log "Check interval: ${CHECK_INTERVAL}s"
log "Crash threshold: ${MAX_CRASHES_PER_HOUR}/hour"
log "=========================================="

restart_count=0

while true; do
    # Check if mdai-system service is active
    if ! systemctl is-active --quiet mdai-system; then
        log "⚠️  mdai-system is DOWN!"
        
        # Record crash time
        CRASH_TIMES+=("$(date +%s)")
        recent_crashes=$(count_recent_crashes)
        
        log "   Recent crashes in last hour: $recent_crashes"
        
        # Check if in crash loop
        if (( recent_crashes >= MAX_CRASHES_PER_HOUR )); then
            log "🚨 CRASH LOOP DETECTED! ($recent_crashes crashes in last hour)"
            log "   Waiting 60 seconds before next restart attempt..."
            log "   Check $CRASH_LOG for crash details"
            sleep 60
        fi
        
        # Check if it's in failed state
        status=$(systemctl is-failed mdai-system 2>/dev/null)
        if [ "$status" = "failed" ]; then
            log "   Service in failed state, resetting..."
            systemctl reset-failed mdai-system 2>/dev/null
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
            
            # Check for recent crash log
            if [ -f "$CRASH_LOG" ]; then
                log "   Last crash info:"
                tail -5 "$CRASH_LOG" | while read line; do
                    log "     $line"
                done
            fi
        fi
    fi
    
    sleep $CHECK_INTERVAL
done
