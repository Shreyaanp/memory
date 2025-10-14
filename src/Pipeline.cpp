#include "Pipeline.hpp"
#include <iostream>

namespace mdai {

Pipeline::Pipeline(const std::string& name, 
                   DynamicRingBuffer* ring_buffer,
                   ConsumerMode mode)
    : name_(name), ring_buffer_(ring_buffer), mode_(mode) {
}

Pipeline::~Pipeline() {
    stop();
}

void Pipeline::add_stage(std::unique_ptr<PipelineStage> stage) {
    if (running_.load()) {
        report_error("Cannot add stage while pipeline is running");
        return;
    }
    stages_.push_back(std::move(stage));
}

bool Pipeline::start() {
    if (running_.load()) {
        report_error("Pipeline already running");
        return false;
    }
    
    if (stages_.empty()) {
        report_error("No stages in pipeline");
        return false;
    }
    
    // Call on_start for all stages
    for (auto& stage : stages_) {
        stage->on_start();
    }
    
    running_.store(true);
    thread_ = std::make_unique<std::thread>(&Pipeline::pipeline_loop, this);
    
    std::cout << "[Pipeline " << name_ << "] Started with " 
              << stages_.size() << " stages" << std::endl;
    
    return true;
}

void Pipeline::stop() {
    if (!running_.load()) {
        return;
    }
    
    running_.store(false);
    
    if (thread_ && thread_->joinable()) {
        thread_->join();
    }
    
    // Call on_stop for all stages
    for (auto& stage : stages_) {
        stage->on_stop();
    }
    
    std::cout << "[Pipeline " << name_ << "] Stopped. "
              << "Processed: " << frames_processed_.load()
              << ", Dropped: " << frames_dropped_.load() << std::endl;
}

bool Pipeline::is_running() const {
    return running_.load();
}

const std::string& Pipeline::get_name() const {
    return name_;
}

ConsumerMode Pipeline::get_mode() const {
    return mode_;
}

uint64_t Pipeline::get_frames_processed() const {
    return frames_processed_.load();
}

uint64_t Pipeline::get_frames_dropped() const {
    return frames_dropped_.load();
}

void Pipeline::set_error_callback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    error_callback_ = callback;
}

void Pipeline::pipeline_loop() {
    while (running_.load()) {
        FrameBox* frame = nullptr;
        
        // Get frame based on mode
        if (mode_ == ConsumerMode::REALTIME) {
            frame = ring_buffer_->get_latest_frame();
        } else {
            frame = ring_buffer_->get_next_frame(last_sequence_id_);
            
            if (frame) {
                // Check for dropped frames
                uint64_t expected_id = last_sequence_id_ + 1;
                if (frame->sequence_id > expected_id) {
                    uint64_t dropped = frame->sequence_id - expected_id;
                    frames_dropped_.fetch_add(dropped);
                    std::cout << "[Pipeline " << name_ << "] Dropped " 
                             << dropped << " frames" << std::endl;
                }
                last_sequence_id_ = frame->sequence_id;
            }
        }
        
        if (frame) {
            // Process frame through all stages (BLOCKING within pipeline)
            process_frame(frame);
            
            // Release frame when done
            ring_buffer_->release_frame(frame);
            
            frames_processed_.fetch_add(1);
        } else {
            // No frame available, yield CPU
            std::this_thread::yield();
        }
    }
}

void Pipeline::process_frame(FrameBox* frame) {
    for (auto& stage : stages_) {
        try {
            // Each stage processes sequentially (BLOCKING)
            bool continue_pipeline = stage->process(frame);
            
            if (!continue_pipeline) {
                // Stage decided to stop processing this frame
                break;
            }
            
        } catch (const std::exception& e) {
            report_error(std::string("Error in stage '") + stage->get_name() + 
                        "': " + e.what());
            break;
        }
    }
}

void Pipeline::report_error(const std::string& error) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    std::cerr << "[Pipeline " << name_ << " Error] " << error << std::endl;
    if (error_callback_) {
        error_callback_(error);
    }
}

// PipelineManager implementation

PipelineManager::PipelineManager(DynamicRingBuffer* ring_buffer)
    : ring_buffer_(ring_buffer) {
}

PipelineManager::~PipelineManager() {
    stop_all();
}

void PipelineManager::add_pipeline(std::unique_ptr<Pipeline> pipeline) {
    pipelines_.push_back(std::move(pipeline));
}

void PipelineManager::start_all() {
    for (auto& pipeline : pipelines_) {
        pipeline->start();
    }
    std::cout << "[PipelineManager] Started " << pipelines_.size() 
              << " pipelines" << std::endl;
}

void PipelineManager::stop_all() {
    for (auto& pipeline : pipelines_) {
        pipeline->stop();
    }
    std::cout << "[PipelineManager] Stopped all pipelines" << std::endl;
}

Pipeline* PipelineManager::get_pipeline(const std::string& name) {
    for (auto& pipeline : pipelines_) {
        if (pipeline->get_name() == name) {
            return pipeline.get();
        }
    }
    return nullptr;
}

const std::vector<std::unique_ptr<Pipeline>>& PipelineManager::get_pipelines() const {
    return pipelines_;
}

} // namespace mdai


