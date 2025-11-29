#include "FrameBox.hpp"

namespace mdai {

FrameBox::FrameBox()
    : sequence_id(0),
      timestamp(0.0),
      ir_width(0),
      ir_height(0),
      exposure(0.0f),
      gain(0.0f),
      metadata(),
      ref_count(0)
{
}

FrameBox::FrameBox(const FrameBox& other) 
    : sequence_id(other.sequence_id),
      timestamp(other.timestamp),
      ir_data(other.ir_data),
      ir_width(other.ir_width),
      ir_height(other.ir_height),
      exposure(other.exposure),
      gain(other.gain),
      metadata(other.metadata),
      ref_count(0)  // ref_count is NOT copied
{
}

FrameBox::FrameBox(FrameBox&& other) noexcept
    : sequence_id(other.sequence_id),
      timestamp(other.timestamp),
      ir_data(std::move(other.ir_data)),
      ir_width(other.ir_width),
      ir_height(other.ir_height),
      exposure(other.exposure),
      gain(other.gain),
      metadata(std::move(other.metadata)),
      ref_count(other.ref_count.load())
{
}

FrameBox& FrameBox::operator=(const FrameBox& other) {
    if (this != &other) {
        sequence_id = other.sequence_id;
        timestamp = other.timestamp;
        ir_data = other.ir_data;
        ir_width = other.ir_width;
        ir_height = other.ir_height;
        exposure = other.exposure;
        gain = other.gain;
        metadata = other.metadata;
        ref_count.store(0);
    }
    return *this;
}

FrameBox& FrameBox::operator=(FrameBox&& other) noexcept {
    if (this != &other) {
        sequence_id = other.sequence_id;
        timestamp = other.timestamp;
        ir_data = std::move(other.ir_data);
        ir_width = other.ir_width;
        ir_height = other.ir_height;
        exposure = other.exposure;
        gain = other.gain;
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
    return {ir_width, ir_height};
}

bool FrameBox::is_valid() const {
    return !ir_data.empty() && ir_width > 0 && ir_height > 0;
}

void FrameBox::acquire() {
    ref_count.fetch_add(1, std::memory_order_relaxed);
}

int FrameBox::release() {
    return ref_count.fetch_sub(1, std::memory_order_relaxed) - 1;
}

// ===== IR ACCESS METHODS =====

#ifdef HAVE_OPENCV
cv::Mat FrameBox::get_ir_mat() const {
    if (ir_data.empty() || ir_width <= 0 || ir_height <= 0) {
        return cv::Mat();
    }
    
    // Create OpenCV Mat from IR data (8-bit grayscale)
    return cv::Mat(ir_height, ir_width, CV_8UC1, 
                   const_cast<uint8_t*>(ir_data.data()));
}

cv::Mat FrameBox::get_ir_as_bgr() const {
    cv::Mat gray = get_ir_mat();
    if (gray.empty()) {
        return cv::Mat();
    }
    
    // Convert grayscale IR to BGR for MediaPipe
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
}
#endif

const uint8_t* FrameBox::get_ir_ptr() const {
    return ir_data.empty() ? nullptr : ir_data.data();
}

const std::vector<uint8_t>& FrameBox::get_ir_vector() const {
    return ir_data;
}

std::tuple<const void*, int, int, const char*> FrameBox::get_ir_numpy() const {
    if (ir_data.empty() || ir_width <= 0 || ir_height <= 0) {
        return {nullptr, 0, 0, "uint8"};
    }
    return {ir_data.data(), ir_width, ir_height, "uint8"};
}

std::string FrameBox::to_json() const {
    std::string json = "{";
    json += "\"sequence_id\":" + std::to_string(sequence_id) + ",";
    json += "\"timestamp\":" + std::to_string(timestamp) + ",";
    json += "\"ir_width\":" + std::to_string(ir_width) + ",";
    json += "\"ir_height\":" + std::to_string(ir_height) + ",";
    json += "\"has_ir\":" + std::string(ir_data.empty() ? "false" : "true") + ",";
    json += "\"face_detected\":" + std::string(metadata.face_detected ? "true" : "false");
    if (metadata.face_detected) {
        json += ",\"face_x\":" + std::to_string(metadata.face_x);
        json += ",\"face_y\":" + std::to_string(metadata.face_y);
        json += ",\"face_w\":" + std::to_string(metadata.face_w);
        json += ",\"face_h\":" + std::to_string(metadata.face_h);
        json += ",\"confidence\":" + std::to_string(metadata.face_detection_confidence);
        json += ",\"landmarks_count\":" + std::to_string(metadata.landmarks.size());
    }
    json += "}";
    return json;
}

} // namespace mdai
