#!/bin/bash
#
# Complete setup and conversion script for BPU model conversion
# This installs Docker, pulls Horizon toolchain, and converts the model
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=============================================="
echo "BPU Model Conversion - Complete Setup"
echo "=============================================="
echo ""

# Step 1: Check/Install Docker
echo "[Step 1/5] Checking Docker installation..."

if ! command -v docker &> /dev/null; then
    echo "Docker not found. Installing..."
    
    # Install Docker
    sudo apt-get update
    sudo apt-get install -y \
        apt-transport-https \
        ca-certificates \
        curl \
        gnupg \
        lsb-release
    
    # Add Docker GPG key
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
    
    # Add Docker repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # Install Docker Engine
    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io
    
    # Add current user to docker group
    sudo usermod -aG docker $USER
    
    echo "✅ Docker installed!"
    echo "NOTE: You may need to log out and back in for group changes to take effect."
    echo "      Or run: newgrp docker"
else
    echo "✅ Docker is already installed: $(docker --version)"
fi

# Step 2: Start Docker service
echo ""
echo "[Step 2/5] Starting Docker service..."
sudo systemctl start docker || true
sudo systemctl enable docker || true
echo "✅ Docker service started"

# Step 3: Pull Horizon toolchain Docker image
echo ""
echo "[Step 3/5] Pulling Horizon AI Toolchain Docker image..."
echo "This may take several minutes (image is ~10GB)..."

# Try different Horizon toolchain images
HORIZON_IMAGES=(
    "openexplorer/ai_toolchain_ubuntu_20_x3:v2.6.2b"
    "openexplorer/ai_toolchain_ubuntu_20_x5:v3.0.0"
    "openexplorer/ai_toolchain:latest"
)

DOCKER_IMAGE=""
for img in "${HORIZON_IMAGES[@]}"; do
    echo "Trying to pull: ${img}"
    if sudo docker pull "${img}" 2>/dev/null; then
        DOCKER_IMAGE="${img}"
        echo "✅ Successfully pulled: ${img}"
        break
    else
        echo "   Image not available: ${img}"
    fi
done

if [ -z "${DOCKER_IMAGE}" ]; then
    echo ""
    echo "⚠️  Could not pull official Horizon Docker images."
    echo ""
    echo "The Horizon AI toolchain may require registration at:"
    echo "  https://developer.horizon.ai/"
    echo ""
    echo "Alternative: You can try the conversion on the RDK device directly"
    echo "if it has the toolchain pre-installed."
    echo ""
    echo "Or download the toolchain manually from Horizon's developer portal."
    exit 1
fi

# Step 4: Generate calibration data
echo ""
echo "[Step 4/5] Checking calibration data..."

if [ ! -d "${SCRIPT_DIR}/calibration_data" ] || [ -z "$(ls -A ${SCRIPT_DIR}/calibration_data 2>/dev/null)" ]; then
    echo "Generating calibration data..."
    cd "${SCRIPT_DIR}"
    python3 generate_calibration_data.py
else
    echo "✅ Calibration data already exists"
fi

# Step 5: Run conversion in Docker
echo ""
echo "[Step 5/5] Running BPU conversion in Docker container..."
echo ""

mkdir -p "${SCRIPT_DIR}/output"

sudo docker run --rm \
    -v "${SCRIPT_DIR}":/workspace \
    -v "${SCRIPT_DIR}/..":/models \
    -w /workspace \
    "${DOCKER_IMAGE}" \
    bash -c "
        echo 'Inside Horizon Docker container'
        echo 'Model: /models/face_landmark.tflite'
        
        # Check for hb_mapper
        if command -v hb_mapper &> /dev/null; then
            echo 'Using hb_mapper for conversion'
            hb_mapper makertbin --config convert_config.yaml
        elif command -v hb_perf &> /dev/null; then
            echo 'Using hb_perf for conversion'
            hb_perf convert --config convert_config.yaml  
        else
            echo 'ERROR: No Horizon conversion tool found!'
            echo 'Available commands:'
            ls /usr/bin/hb* 2>/dev/null || echo 'No hb_* commands found'
            exit 1
        fi
    "

echo ""
echo "=============================================="
echo "Conversion Complete!"
echo "=============================================="

if [ -f "${SCRIPT_DIR}/output/face_landmark.bin" ]; then
    echo "✅ BPU model created successfully!"
    echo ""
    ls -la "${SCRIPT_DIR}/output/"
    echo ""
    echo "Next steps:"
    echo "1. Copy face_landmark.bin to your RDK device:"
    echo "   scp ${SCRIPT_DIR}/output/face_landmark.bin user@rdk:/path/to/models/"
    echo ""
    echo "2. Use the model with hobot_dnn or libdnn API"
else
    echo "⚠️  Conversion may have failed. Check output directory:"
    ls -la "${SCRIPT_DIR}/output/" 2>/dev/null || echo "Output directory is empty"
fi

