#!/bin/bash
#
# Convert TFLite model to Horizon BPU format
# This script should be run inside the Horizon Docker container
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_DIR="${SCRIPT_DIR}"
MODEL_NAME="face_landmark"
TFLITE_MODEL="${SCRIPT_DIR}/../face_landmark.tflite"

echo "=============================================="
echo "Horizon BPU Model Conversion"
echo "=============================================="
echo ""
echo "Model: ${TFLITE_MODEL}"
echo "Output: ${MODEL_DIR}/${MODEL_NAME}.bin"
echo ""

# Check if running inside Horizon Docker
if ! command -v hb_mapper &> /dev/null; then
    echo "ERROR: hb_mapper not found!"
    echo ""
    echo "This script must be run inside the Horizon Docker container."
    echo ""
    echo "Steps to run:"
    echo "1. Pull the Horizon Docker image:"
    echo "   docker pull openexplorer/ai_toolchain_ubuntu_20_x3:v2.6.2b"
    echo ""
    echo "2. Run the container:"
    echo "   docker run -it --rm \\"
    echo "     -v ${SCRIPT_DIR}:/workspace \\"
    echo "     -v ${SCRIPT_DIR}/..:/models \\"
    echo "     openexplorer/ai_toolchain_ubuntu_20_x3:v2.6.2b bash"
    echo ""
    echo "3. Inside container, run:"
    echo "   cd /workspace && bash convert_to_bpu.sh"
    echo ""
    exit 1
fi

# Step 1: Generate calibration data if not exists
if [ ! -d "${MODEL_DIR}/calibration_data" ] || [ -z "$(ls -A ${MODEL_DIR}/calibration_data 2>/dev/null)" ]; then
    echo "Generating calibration data..."
    cd "${MODEL_DIR}"
    python3 generate_calibration_data.py
fi

# Step 2: Create the conversion YAML config
cat > "${MODEL_DIR}/convert_config.yaml" << 'EOF'
# Horizon BPU Conversion Config for face_landmark.tflite

model_parameters:
  tflite_model: '/models/face_landmark.tflite'
  march: 'bayes'  # Use 'bernoulli2' for X3, 'bayes' for X5
  layer_out_dump: False
  working_dir: '/workspace/output'
  output_model_file_prefix: 'face_landmark'

input_parameters:
  input_name: 'input_1'
  input_type_rt: 'featuremap'
  input_layout_rt: 'NHWC'
  input_type_train: 'rgb'
  input_layout_train: 'NHWC'
  input_shape: '1x192x192x3'
  norm_type: 'data_scale'
  scale_value: '0.003921568627'

calibration_parameters:
  cal_data_dir: '/workspace/calibration_data'
  cal_data_type: 'float32'
  calibration_type: 'default'

compiler_parameters:
  compile_mode: 'latency'
  optimize_level: 'O3'
  debug: False
  core_num: 2
EOF

echo "Created conversion config: ${MODEL_DIR}/convert_config.yaml"

# Step 3: Create output directory
mkdir -p "${MODEL_DIR}/output"

# Step 4: Run the conversion
echo ""
echo "Running BPU model conversion..."
echo "This may take several minutes..."
echo ""

cd "${MODEL_DIR}"

# Method 1: Using hb_mapper (newer toolchain)
if command -v hb_mapper &> /dev/null; then
    hb_mapper makertbin --config convert_config.yaml
    
# Method 2: Using hb_perf for older toolchain
elif command -v hb_perf &> /dev/null; then
    hb_perf convert --config convert_config.yaml
fi

# Step 5: Check output
echo ""
echo "=============================================="
echo "Conversion Complete!"
echo "=============================================="

if [ -f "${MODEL_DIR}/output/${MODEL_NAME}.bin" ]; then
    echo "✅ BPU model created: ${MODEL_DIR}/output/${MODEL_NAME}.bin"
    ls -la "${MODEL_DIR}/output/"
else
    echo "⚠️  Output file not found. Check the conversion log for errors."
    ls -la "${MODEL_DIR}/output/" 2>/dev/null || echo "Output directory is empty."
fi

echo ""
echo "Next steps:"
echo "1. Copy ${MODEL_NAME}.bin to your RDK device"
echo "2. Use libdnn or hobot_dnn API to run inference"

