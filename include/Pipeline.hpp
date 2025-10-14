#pragma once

#include "FrameBox.hpp"
#include "RingBuffer.hpp"
#include <thread>
#include <atomic>
#include <functional>
#include <vector>
#include <memory>
#include <string>

namespace mdai {

/**
 * @brief Consumer mode for accessing frames from ring buffer
 */
enum class ConsumerMode {
    REALTIME,    // Always get latest frame, skip older ones
    SEQUENTIAL   // Process frames in order, detect drops
};

/**
 * @brief Base class for pipeline stages
 * 
 * A Pipeline contains multiple stages that execute sequentially
 * within the pipeline but independently of other pipelines.
 */
class PipelineStage {
public:
    virtual ~PipelineStage() = default;
    
    /**
     * @brief Process a single frame
     * @param frame Pointer to FrameBox to process
     * @return true to continue to next stage, false to stop pipeline for this frame
     */
    virtual bool process(FrameBox* frame) = 0;
    
    /**
     * @brief Get stage name
     */
    virtual std::string get_name() const = 0;
    
    /**
     * @brief Called when pipeline starts
     */
    virtual void on_start() {}
    
    /**
     * @brief Called when pipeline stops
     */
    virtual void on_stop() {}
};

/**
 * @brief Pipeline - executes stages sequentially on frames
 * 
 * Key features:
 * - Stages within a pipeline execute sequentially (blocking within pipeline)
 * - Multiple pipelines run independently (non-blocking across pipelines)
 * - Can operate in realtime or sequential mode
 */
class Pipeline {
public:
    /**
     * @brief Constructor
     * @param name Pipeline name for identification
     * @param ring_buffer Pointer to ring buffer to read frames
     * @param mode Consumer mode (realtime or sequential)
     */
    Pipeline(const std::string& name, 
             DynamicRingBuffer* ring_buffer, 
             ConsumerMode mode = ConsumerMode::REALTIME);
    
    /**
     * @brief Destructor - stops pipeline
     */
    ~Pipeline();
    
    /**
     * @brief Add a stage to the pipeline
     * @param stage Unique pointer to stage (ownership transferred)
     */
    void add_stage(std::unique_ptr<PipelineStage> stage);
    
    /**
     * @brief Start the pipeline thread
     * @return true if started successfully
     */
    bool start();
    
    /**
     * @brief Stop the pipeline thread
     */
    void stop();
    
    /**
     * @brief Check if pipeline is running
     */
    bool is_running() const;
    
    /**
     * @brief Get pipeline name
     */
    const std::string& get_name() const;
    
    /**
     * @brief Get consumer mode
     */
    ConsumerMode get_mode() const;
    
    /**
     * @brief Get total frames processed
     */
    uint64_t get_frames_processed() const;
    
    /**
     * @brief Get total frames dropped (for sequential mode)
     */
    uint64_t get_frames_dropped() const;
    
    /**
     * @brief Set error callback
     */
    void set_error_callback(std::function<void(const std::string&)> callback);
    
private:
    std::string name_;
    DynamicRingBuffer* ring_buffer_;
    ConsumerMode mode_;
    
    std::vector<std::unique_ptr<PipelineStage>> stages_;
    
    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> running_{false};
    
    // For sequential mode
    uint64_t last_sequence_id_{0};
    
    // Statistics
    std::atomic<uint64_t> frames_processed_{0};
    std::atomic<uint64_t> frames_dropped_{0};
    
    std::function<void(const std::string&)> error_callback_;
    std::mutex callback_mutex_;
    
    /**
     * @brief Main pipeline loop
     */
    void pipeline_loop();
    
    /**
     * @brief Process frame through all stages
     * @param frame Pointer to frame to process
     */
    void process_frame(FrameBox* frame);
    
    /**
     * @brief Report error
     */
    void report_error(const std::string& error);
};

/**
 * @brief Pipeline manager - manages multiple independent pipelines
 */
class PipelineManager {
public:
    /**
     * @brief Constructor
     * @param ring_buffer Pointer to ring buffer
     */
    explicit PipelineManager(DynamicRingBuffer* ring_buffer);
    
    /**
     * @brief Destructor - stops all pipelines
     */
    ~PipelineManager();
    
    /**
     * @brief Add a pipeline
     * @param pipeline Unique pointer to pipeline (ownership transferred)
     */
    void add_pipeline(std::unique_ptr<Pipeline> pipeline);
    
    /**
     * @brief Start all pipelines
     */
    void start_all();
    
    /**
     * @brief Stop all pipelines
     */
    void stop_all();
    
    /**
     * @brief Get pipeline by name
     * @param name Pipeline name
     * @return Pointer to pipeline, or nullptr if not found
     */
    Pipeline* get_pipeline(const std::string& name);
    
    /**
     * @brief Get all pipelines
     */
    const std::vector<std::unique_ptr<Pipeline>>& get_pipelines() const;
    
private:
    DynamicRingBuffer* ring_buffer_;
    std::vector<std::unique_ptr<Pipeline>> pipelines_;
};

} // namespace mdai


