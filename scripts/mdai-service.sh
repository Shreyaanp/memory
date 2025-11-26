#!/bin/bash
# MDai Service Management Script
# Usage: ./mdai-service.sh [start|stop|restart|status|logs|build-restart]

set -e

SERVICE_NAME="mdai-system"
LOG_FILE="/tmp/mdai/rdk.log"
BUILD_DIR="/home/mercleDev/codebase/build"

case "$1" in
    start)
        echo "🚀 Starting MDai service..."
        sudo systemctl start $SERVICE_NAME
        sleep 3
        sudo systemctl status $SERVICE_NAME --no-pager
        ;;
    stop)
        echo "🛑 Stopping MDai service..."
        sudo systemctl stop $SERVICE_NAME
        echo "✅ Service stopped"
        ;;
    restart)
        echo "🔄 Restarting MDai service..."
        sudo systemctl restart $SERVICE_NAME
        sleep 8
        if systemctl is-active --quiet $SERVICE_NAME; then
            echo "✅ Service restarted successfully"
            tail -10 $LOG_FILE
        else
            echo "❌ Service failed to start"
            sudo journalctl -u $SERVICE_NAME -n 20 --no-pager
            exit 1
        fi
        ;;
    status)
        sudo systemctl status $SERVICE_NAME --no-pager
        echo ""
        echo "📊 Recent logs:"
        tail -15 $LOG_FILE 2>/dev/null || echo "No log file found"
        ;;
    logs)
        echo "📜 Following MDai logs (Ctrl+C to stop)..."
        tail -f $LOG_FILE
        ;;
    build-restart)
        echo "🔨 Building and restarting MDai..."
        
        # Stop service first
        echo "🛑 Stopping service..."
        sudo systemctl stop $SERVICE_NAME
        sleep 2
        
        # Build
        echo "🔨 Building..."
        cd /home/mercleDev/codebase
        mkdir -p build && cd build
        cmake .. && make -j$(nproc)
        
        if [ $? -eq 0 ]; then
            echo "✅ Build successful"
            
            # Restart service
            echo "🚀 Starting service..."
            sudo systemctl start $SERVICE_NAME
            sleep 8
            
            if systemctl is-active --quiet $SERVICE_NAME; then
                echo "✅ Service running"
                tail -15 $LOG_FILE
            else
                echo "❌ Service failed to start"
                sudo journalctl -u $SERVICE_NAME -n 20 --no-pager
                exit 1
            fi
        else
            echo "❌ Build failed"
            exit 1
        fi
        ;;
    *)
        echo "MDai Service Management"
        echo ""
        echo "Usage: $0 {start|stop|restart|status|logs|build-restart}"
        echo ""
        echo "Commands:"
        echo "  start         - Start the MDai service"
        echo "  stop          - Stop the MDai service"
        echo "  restart       - Restart the MDai service"
        echo "  status        - Show service status and recent logs"
        echo "  logs          - Follow the log file"
        echo "  build-restart - Build the project and restart service"
        exit 1
        ;;
esac



