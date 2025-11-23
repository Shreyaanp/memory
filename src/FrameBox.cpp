#include "FrameBox.hpp"

namespace mdai {

FrameBox::FrameBox()
    : sequence_id(0),
      time_depth(0.0),
      time_color(0.0),
      time_ir_left(0.0),
      time_ir_right(0.0),
      depth_width(0),
      depth_height(0),
      color_width(0),
      color_height(0),
      ir_width(0),
      ir_height(0),
      color_bytes_per_pixel(3),
      depth_scale(0.001f),
      exposure(0.0f),
      gain(0.0f),
      emitter_state(0),
      laser_power(0.0f),
      metadata(),
      ref_count(0)
{
}

FrameBox::FrameBox(const FrameBox& other) 
    : sequence_id(other.sequence_id),
      time_depth(other.time_depth),
      time_color(other.time_color),
      time_ir_left(other.time_ir_left),
      time_ir_right(other.time_ir_right),
      depth_data(other.depth_data),
      color_data(other.color_data),
      ir_left_data(other.ir_left_data),
      ir_right_data(other.ir_right_data),
      depth_width(other.depth_width),
      depth_height(other.depth_height),
      color_width(other.color_width),
      color_height(other.color_height),
      ir_width(other.ir_width),
      ir_height(other.ir_height),
      color_bytes_per_pixel(other.color_bytes_per_pixel),
      depth_intrinsics(other.depth_intrinsics),
      color_intrinsics(other.color_intrinsics),
      ir_intrinsics(other.ir_intrinsics),
      extrinsics_depth_to_color(other.extrinsics_depth_to_color),
      depth_scale(other.depth_scale),
      exposure(other.exposure),
      gain(other.gain),
      emitter_state(other.emitter_state),
      laser_power(other.laser_power),
      filter_settings(other.filter_settings),
      metadata(other.metadata),
      ref_count(0)
{
    // Note: ref_count is NOT copied, starts at 0
}

FrameBox::FrameBox(FrameBox&& other) noexcept
    : sequence_id(other.sequence_id),
      time_depth(other.time_depth),
      time_color(other.time_color),
      time_ir_left(other.time_ir_left),
      time_ir_right(other.time_ir_right),
      depth_data(std::move(other.depth_data)),
      color_data(std::move(other.color_data)),
      ir_left_data(std::move(other.ir_left_data)),
      ir_right_data(std::move(other.ir_right_data)),
      depth_width(other.depth_width),
      depth_height(other.depth_height),
      color_width(other.color_width),
      color_height(other.color_height),
      ir_width(other.ir_width),
      ir_height(other.ir_height),
      color_bytes_per_pixel(other.color_bytes_per_pixel),
      depth_intrinsics(other.depth_intrinsics),
      color_intrinsics(other.color_intrinsics),
      ir_intrinsics(other.ir_intrinsics),
      extrinsics_depth_to_color(other.extrinsics_depth_to_color),
      depth_scale(other.depth_scale),
      exposure(other.exposure),
      gain(other.gain),
      emitter_state(other.emitter_state),
      laser_power(other.laser_power),
      filter_settings(other.filter_settings),
      metadata(std::move(other.metadata)),
      ref_count(other.ref_count.load())
{
}

FrameBox& FrameBox::operator=(const FrameBox& other) {
    if (this != &other) {
        sequence_id = other.sequence_id;
        time_depth = other.time_depth;
        time_color = other.time_color;
        time_ir_left = other.time_ir_left;
        time_ir_right = other.time_ir_right;
        depth_data = other.depth_data;
        color_data = other.color_data;
        ir_left_data = other.ir_left_data;
        ir_right_data = other.ir_right_data;
        depth_width = other.depth_width;
        depth_height = other.depth_height;
        color_width = other.color_width;
        color_height = other.color_height;
        ir_width = other.ir_width;
        ir_height = other.ir_height;
        color_bytes_per_pixel = other.color_bytes_per_pixel;
        depth_intrinsics = other.depth_intrinsics;
        color_intrinsics = other.color_intrinsics;
        ir_intrinsics = other.ir_intrinsics;
        extrinsics_depth_to_color = other.extrinsics_depth_to_color;
        depth_scale = other.depth_scale;
        exposure = other.exposure;
        gain = other.gain;
        emitter_state = other.emitter_state;
        laser_power = other.laser_power;
        filter_settings = other.filter_settings;
        metadata = other.metadata;
        ref_count.store(0);
    }
    return *this;
}

FrameBox& FrameBox::operator=(FrameBox&& other) noexcept {
    if (this != &other) {
        sequence_id = other.sequence_id;
        time_depth = other.time_depth;
        time_color = other.time_color;
        time_ir_left = other.time_ir_left;
        time_ir_right = other.time_ir_right;
        depth_data = std::move(other.depth_data);
        color_data = std::move(other.color_data);
        ir_left_data = std::move(other.ir_left_data);
        ir_right_data = std::move(other.ir_right_data);
        depth_width = other.depth_width;
        depth_height = other.depth_height;
        color_width = other.color_width;
        color_height = other.color_height;
        ir_width = other.ir_width;
        ir_height = other.ir_height;
        color_bytes_per_pixel = other.color_bytes_per_pixel;
        depth_intrinsics = other.depth_intrinsics;
        color_intrinsics = other.color_intrinsics;
        ir_intrinsics = other.ir_intrinsics;
        extrinsics_depth_to_color = other.extrinsics_depth_to_color;
        depth_scale = other.depth_scale;
        exposure = other.exposure;
        gain = other.gain;
        emitter_state = other.emitter_state;
        laser_power = other.laser_power;
        filter_settings = other.filter_settings;
        metadata = std::move(other.metadata);
        ref_count.store(other.ref_count.load());
    }
    return *this;
}

void FrameBox::notify_stage_complete(const std::string& stage_name) {
    std::lock_guard<std::mutex> lock(stage_mutex);
    completed_stages.insert(stage_name);
    stage_cv.notify_all();
}

bool FrameBox::wait_for_stage(const std::string& stage_name, int timeout_ms) {
    std::unique_lock<std::mutex> lock(stage_mutex);
    return stage_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
        [&]{ return completed_stages.count(stage_name) > 0; });
}

bool FrameBox::is_stage_complete(const std::string& stage_name) const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(stage_mutex));
    return completed_stages.count(stage_name) > 0;
}

std::pair<int, int> FrameBox::get_dimensions() const {
    if (depth_width > 0 && depth_height > 0) {
        return {depth_width, depth_height};
    }
    if (color_width > 0 && color_height > 0) {
        return {color_width, color_height};
    }
    return {0, 0};
}

bool FrameBox::is_valid() const {
    return !depth_data.empty() || !color_data.empty();
}

float FrameBox::get_depth_at(int x, int y) const {
    if (depth_data.empty() || depth_width == 0 || depth_height == 0) {
        return 0.0f;
    }
    
    if (x < 0 || x >= depth_width || y < 0 || y >= depth_height) {
        return 0.0f;
    }
    
    // Get depth value and convert to meters
    int index = y * depth_width + x;
    return depth_data[index] * depth_scale;
}

std::array<float, 3> FrameBox::deproject_pixel_to_point(int x, int y) const {
    std::array<float, 3> point = {0.0f, 0.0f, 0.0f};
    
    if (depth_data.empty() || depth_width == 0 || depth_height == 0) {
        return point;
    }
    
    float depth_value = get_depth_at(x, y);
    if (depth_value == 0.0f) return point;
    
    float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
    rs2_deproject_pixel_to_point(point.data(), &depth_intrinsics, pixel, depth_value);
    
    return point;
}

void FrameBox::acquire() {
    ref_count.fetch_add(1, std::memory_order_relaxed);
}

int FrameBox::release() {
    return ref_count.fetch_sub(1, std::memory_order_relaxed) - 1;
}

// ===== UNIFIED ACCESS SYSTEM IMPLEMENTATION =====

#ifdef HAVE_OPENCV
cv::Mat FrameBox::get_depth_mat() const {
    if (depth_data.empty() || depth_width == 0 || depth_height == 0) {
        return cv::Mat();
    }
    
    // Create OpenCV Mat from depth data (16-bit unsigned)
    return cv::Mat(depth_height, depth_width, CV_16UC1, 
                   const_cast<uint16_t*>(depth_data.data()));
}

cv::Mat FrameBox::get_color_mat() const {
    if (color_data.empty() || color_width == 0 || color_height == 0) {
        return cv::Mat();
    }
    
    // Create OpenCV Mat from color data (8-bit BGR)
    return cv::Mat(color_height, color_width, CV_8UC3, 
                   const_cast<uint8_t*>(color_data.data()));
}
#endif

const uint16_t* FrameBox::get_depth_ptr() const {
    return depth_data.empty() ? nullptr : depth_data.data();
}

const uint8_t* FrameBox::get_color_ptr() const {
    return color_data.empty() ? nullptr : color_data.data();
}

const std::vector<uint16_t>& FrameBox::get_depth_vector() const {
    return depth_data;
}

const std::vector<uint8_t>& FrameBox::get_color_vector() const {
    return color_data;
}

const std::vector<uint8_t>& FrameBox::get_ir_left_vector() const {
    return ir_left_data;
}

const std::vector<uint8_t>& FrameBox::get_ir_right_vector() const {
    return ir_right_data;
}

std::tuple<const void*, int, int, const char*> FrameBox::get_depth_numpy() const {
    if (depth_data.empty() || depth_width == 0 || depth_height == 0) {
        return {nullptr, 0, 0, "uint16"};
    }
    return {depth_data.data(), depth_width, depth_height, "uint16"};
}

std::tuple<const void*, int, int, const char*> FrameBox::get_color_numpy() const {
    if (color_data.empty() || color_width == 0 || color_height == 0) {
        return {nullptr, 0, 0, "uint8"};
    }
    return {color_data.data(), color_width, color_height, "uint8"};
}

std::string FrameBox::to_json() const {
    std::string json = "{";
    json += "\"sequence_id\":" + std::to_string(sequence_id) + ",";
    json += "\"time_depth\":" + std::to_string(time_depth) + ",";
    json += "\"time_color\":" + std::to_string(time_color) + ",";
    json += "\"depth_width\":" + std::to_string(depth_width) + ",";
    json += "\"depth_height\":" + std::to_string(depth_height) + ",";
    json += "\"color_width\":" + std::to_string(color_width) + ",";
    json += "\"color_height\":" + std::to_string(color_height) + ",";
    json += "\"depth_scale\":" + std::to_string(depth_scale) + ",";
    json += "\"has_depth\":" + std::string(depth_data.empty() ? "false" : "true") + ",";
    json += "\"has_color\":" + std::string(color_data.empty() ? "false" : "true") + ",";
    json += "\"has_ir_left\":" + std::string(ir_left_data.empty() ? "false" : "true") + ",";
    json += "\"has_ir_right\":" + std::string(ir_right_data.empty() ? "false" : "true");
    json += "}";
    return json;
}

std::vector<uint8_t> FrameBox::to_binary() const {
    std::vector<uint8_t> binary;
    
    // Header: sequence_id, timestamps, dimensions
    binary.resize(sizeof(uint64_t) * 3 + sizeof(int) * 4 + sizeof(float));
    size_t offset = 0;
    
    // Sequence ID
    std::memcpy(binary.data() + offset, &sequence_id, sizeof(uint64_t));
    offset += sizeof(uint64_t);
    
    // Timestamps
    std::memcpy(binary.data() + offset, &time_depth, sizeof(double));
    offset += sizeof(double);
    std::memcpy(binary.data() + offset, &time_color, sizeof(double));
    offset += sizeof(double);
    
    // Dimensions
    std::memcpy(binary.data() + offset, &depth_width, sizeof(int));
    offset += sizeof(int);
    std::memcpy(binary.data() + offset, &depth_height, sizeof(int));
    offset += sizeof(int);
    std::memcpy(binary.data() + offset, &color_width, sizeof(int));
    offset += sizeof(int);
    std::memcpy(binary.data() + offset, &color_height, sizeof(int));
    offset += sizeof(int);
    
    // Depth scale
    std::memcpy(binary.data() + offset, &depth_scale, sizeof(float));
    offset += sizeof(float);
    
    // Data sizes
    size_t depth_size = depth_data.size() * sizeof(uint16_t);
    size_t color_size = color_data.size() * sizeof(uint8_t);
    size_t ir_left_size = ir_left_data.size() * sizeof(uint8_t);
    size_t ir_right_size = ir_right_data.size() * sizeof(uint8_t);
    
    binary.resize(offset + sizeof(size_t) * 4 + depth_size + color_size + ir_left_size + ir_right_size);
    
    // Data sizes
    std::memcpy(binary.data() + offset, &depth_size, sizeof(size_t));
    offset += sizeof(size_t);
    std::memcpy(binary.data() + offset, &color_size, sizeof(size_t));
    offset += sizeof(size_t);
    std::memcpy(binary.data() + offset, &ir_left_size, sizeof(size_t));
    offset += sizeof(size_t);
    std::memcpy(binary.data() + offset, &ir_right_size, sizeof(size_t));
    offset += sizeof(size_t);
    
    // Data
    if (depth_size > 0) {
        std::memcpy(binary.data() + offset, depth_data.data(), depth_size);
        offset += depth_size;
    }
    if (color_size > 0) {
        std::memcpy(binary.data() + offset, color_data.data(), color_size);
        offset += color_size;
    }
    if (ir_left_size > 0) {
        std::memcpy(binary.data() + offset, ir_left_data.data(), ir_left_size);
        offset += ir_left_size;
    }
    if (ir_right_size > 0) {
        std::memcpy(binary.data() + offset, ir_right_data.data(), ir_right_size);
        offset += ir_right_size;
    }
    
    return binary;
}

bool FrameBox::from_binary(const std::vector<uint8_t>& data) {
    if (data.size() < sizeof(uint64_t) * 3 + sizeof(int) * 4 + sizeof(float) + sizeof(size_t) * 4) {
        return false;
    }
    
    size_t offset = 0;
    
    // Sequence ID
    std::memcpy(&sequence_id, data.data() + offset, sizeof(uint64_t));
    offset += sizeof(uint64_t);
    
    // Timestamps
    std::memcpy(&time_depth, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    std::memcpy(&time_color, data.data() + offset, sizeof(double));
    offset += sizeof(double);
    
    // Dimensions
    std::memcpy(&depth_width, data.data() + offset, sizeof(int));
    offset += sizeof(int);
    std::memcpy(&depth_height, data.data() + offset, sizeof(int));
    offset += sizeof(int);
    std::memcpy(&color_width, data.data() + offset, sizeof(int));
    offset += sizeof(int);
    std::memcpy(&color_height, data.data() + offset, sizeof(int));
    offset += sizeof(int);
    
    // Depth scale
    std::memcpy(&depth_scale, data.data() + offset, sizeof(float));
    offset += sizeof(float);
    
    // Data sizes
    size_t depth_size, color_size, ir_left_size, ir_right_size;
    std::memcpy(&depth_size, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    std::memcpy(&color_size, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    std::memcpy(&ir_left_size, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    std::memcpy(&ir_right_size, data.data() + offset, sizeof(size_t));
    offset += sizeof(size_t);
    
    // Check if we have enough data
    if (data.size() < offset + depth_size + color_size + ir_left_size + ir_right_size) {
        return false;
    }
    
    // Load data
    if (depth_size > 0) {
        depth_data.resize(depth_size / sizeof(uint16_t));
        std::memcpy(depth_data.data(), data.data() + offset, depth_size);
        offset += depth_size;
    }
    
    if (color_size > 0) {
        color_data.resize(color_size / sizeof(uint8_t));
        std::memcpy(color_data.data(), data.data() + offset, color_size);
        offset += color_size;
    }
    
    if (ir_left_size > 0) {
        ir_left_data.resize(ir_left_size / sizeof(uint8_t));
        std::memcpy(ir_left_data.data(), data.data() + offset, ir_left_size);
        offset += ir_left_size;
    }
    
    if (ir_right_size > 0) {
        ir_right_data.resize(ir_right_size / sizeof(uint8_t));
        std::memcpy(ir_right_data.data(), data.data() + offset, ir_right_size);
        offset += ir_right_size;
    }
    
    return true;
}

} // namespace mdai
