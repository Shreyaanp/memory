#!/bin/bash
# MDai System Control Script
# Usage: ./mdai.sh [restart|build|stop]

cd /home/mercleDev/codebase

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

kill_all() {
    echo -e "${YELLOW}Stopping mdai-system service...${NC}"
    
    # Stop the systemd service first (prevents auto-restart)
    sudo systemctl stop mdai-system 2>/dev/null
    sudo systemctl stop mdai-watchdog 2>/dev/null
    
    echo -e "${YELLOW}Killing all mdai processes...${NC}"
    sudo pkill -9 mdai_system 2>/dev/null
    pkill -9 mdai_system 2>/dev/null
    sleep 1
    
    # Verify killed
    if pgrep -f "mdai_system$" > /dev/null; then
        echo -e "${RED}Warning: Some processes still running${NC}"
        pgrep -f "mdai_system$"
    else
        echo -e "${GREEN}All processes stopped${NC}"
        echo -e "${YELLOW}Note: Service stopped. Run 'sudo systemctl start mdai-system' to re-enable auto-start${NC}"
    fi
}

start_server() {
    echo -e "${GREEN}Starting mdai_system (manual mode)...${NC}"
    cd /home/mercleDev/codebase/build
    export LD_LIBRARY_PATH=$PWD:/home/mercleDev/codebase/lib/mediapipe:$LD_LIBRARY_PATH
    
    # Clear log
    echo "" > /home/mercleDev/mdai_logs/rdk.log
    
    # Start in background
    ./mdai_system &
    
    sleep 3
    
    if pgrep -f "mdai_system$" > /dev/null; then
        echo -e "${GREEN}✓ Server started (PID: $(pgrep -f 'mdai_system$' | head -1))${NC}"
        echo -e "${GREEN}✓ Logs: tail -f /home/mercleDev/mdai_logs/rdk.log${NC}"
        echo -e "${YELLOW}⚠ Running in MANUAL mode (service stopped)${NC}"
        echo -e "${YELLOW}  To restore auto-start: sudo systemctl start mdai-system${NC}"
    else
        echo -e "${RED}✗ Server failed to start${NC}"
        tail -20 /home/mercleDev/mdai_logs/rdk.log
    fi
}

build_server() {
    echo -e "${YELLOW}Building with 7 parallel jobs...${NC}"
    cd /home/mercleDev/codebase/build
    
    if make -j7 2>&1; then
        echo -e "${GREEN}✓ Build successful${NC}"
        return 0
    else
        echo -e "${RED}✗ Build failed${NC}"
        return 1
    fi
}

case "$1" in
    restart)
        kill_all
        start_server
        ;;
    build)
        kill_all
        if build_server; then
            start_server
        fi
        ;;
    stop)
        kill_all
        ;;
    service)
        # Re-enable systemd service (production mode)
        echo -e "${YELLOW}Stopping manual instance...${NC}"
        sudo pkill -9 mdai_system 2>/dev/null
        sleep 1
        echo -e "${GREEN}Starting mdai-system service...${NC}"
        sudo systemctl start mdai-watchdog
        sudo systemctl start mdai-system
        sleep 2
        if systemctl is-active --quiet mdai-system; then
            echo -e "${GREEN}✓ Service started (auto-restart enabled)${NC}"
            systemctl status mdai-system --no-pager | head -5
        else
            echo -e "${RED}✗ Service failed to start${NC}"
            sudo journalctl -xeu mdai-system --no-pager | tail -10
        fi
        ;;
    status)
        /home/mercleDev/codebase/deployment/status.sh
        ;;
    logs)
        tail -f /home/mercleDev/mdai_logs/system.log
        ;;
    *)
        echo "MDai System Control"
        echo ""
        echo "Usage: ./mdai.sh [command]"
        echo ""
        echo "Commands:"
        echo -e "  ${GREEN}restart${NC}  - Stop service, run latest build manually"
        echo -e "  ${GREEN}build${NC}    - Stop service, rebuild, run manually"
        echo -e "  ${GREEN}stop${NC}     - Stop everything (service + manual)"
        echo -e "  ${GREEN}service${NC}  - Switch back to systemd service (production)"
        echo -e "  ${GREEN}status${NC}   - Show system status"
        echo -e "  ${GREEN}logs${NC}     - Follow log output"
        echo ""
        echo "Modes:"
        echo -e "  ${YELLOW}Manual${NC}   - ./mdai.sh restart (for testing)"
        echo -e "  ${YELLOW}Service${NC}  - ./mdai.sh service (for production, auto-restart)"
        echo ""
        ;;
esac






