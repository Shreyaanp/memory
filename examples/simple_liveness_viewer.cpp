#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "AntiSpoofing.hpp"
#include "FaceDetector.hpp"

using namespace mdai;

class SimpleLivenessViewer {
private:
    // Components
    std::unique_ptr<Producer> producer_;
    std::unique_ptr<DynamicRingBuffer> ring_buffer_;
    std::unique_ptr<FaceDetector> face_detector_;
    std::unique_ptr<QualityGate> quality_gate_;
    std::unique_ptr<AntiSpoofingDetector> anti_spoofing_;
    AntiSpoofingConfig config_;
    
    // State
    bool camera_running_ = false;
    bool show_debug_info_ = true;
    
    // Statistics
    int total_frames_ = 0;
    int face_detected_ = 0;
    int live_detected_ = 0;
    int spoof_detected_ = 0;
    
    // Face detection cache (for frame skipping optimization)
    bool last_face_detected_ = false;
    int last_face_x_ = 0;
    int last_face_y_ = 0;
    int last_face_w_ = 0;
    int last_face_h_ = 0;
    float last_face_confidence_ = 0.0f;
    FrameBoxMetadata::FacialLandmarks last_landmarks_;
    
public:
    SimpleLivenessViewer() {
        
        std::cout << "🎥 Simple Liveness Viewer" << std::endl;
        std::cout << "=========================" << std::endl;
        std::cout << "This viewer shows real-time camera feed with liveness analysis." << std::endl;
        std::cout << "\nControls:" << std::endl;
        std::cout << "  - Press 'C' to start/stop camera" << std::endl;
        std::cout << "  - Press 'D' to toggle debug info" << std::endl;
        std::cout << "  - Press 'R' to reset statistics" << std::endl;
        std::cout << "  - Press 'Q' to quit" << std::endl;
        std::cout << "\nUsing Anti-Spoofing system with:" << std::endl;
        std::cout << "  - Quality Gates" << std::endl;
        std::cout << "  - Face Detection" << std::endl;
        std::cout << "  - Depth Analysis" << std::endl;
        std::cout << "  - IR Texture Analysis" << std::endl;
        std::cout << "  - Cross-Modal Consistency" << std::endl;
        
        // Initialize configuration (optimized for real humans)
        config_.min_lighting_score = 0.3f;         // Lenient lighting
        config_.min_motion_score = 0.25f;          // Lenient sharpness
        config_.min_positioning_score = 0.4f;      // FIXED: Accept off-center faces
        config_.min_synchronization_score = 0.5f;  // Basic sensor sync
        config_.min_stability_score = 0.4f;        // Lenient camera settings
        config_.min_overall_quality = 0.4f;        // Lower quality gate
        config_.min_depth_analysis_score = 0.5f;   // Key anti-spoof metric
        config_.min_ir_texture_score = 0.35f;      // FIXED: Accept human IR variance
        config_.min_temporal_consistency_score = 0.35f;  // FIXED: Don't penalize steady subjects
        config_.min_cross_modal_score = 0.4f;      // Basic cross-modal check
        config_.min_overall_liveness = 0.5f;       // Overall liveness threshold
        config_.min_confidence = 0.5f;             // Lower confidence requirement
    }
    
    void run() {
        // Create the main window
        cv::namedWindow("Liveness Viewer", cv::WINDOW_AUTOSIZE);
        
        // Auto-start camera
        start_camera();
        
        while (true) {
            update_display();
            
            char key = cv::waitKey(30) & 0xFF;
            if (key == 'q' || key == 'Q') {
                break;
            }
            else if (key == 'c' || key == 'C') {
                toggle_camera();
            }
            else if (key == 'd' || key == 'D') {
                show_debug_info_ = !show_debug_info_;
                std::cout << "Debug info: " << (show_debug_info_ ? "ON" : "OFF") << std::endl;
            }
            else if (key == 'r' || key == 'R') {
                reset_statistics();
            }
        }
        
        cleanup();
    }
    
private:
    void update_display() {
        // Create main display
        cv::Mat display = cv::Mat::zeros(600, 1000, CV_8UC3);
        
        if (camera_running_ && ring_buffer_) {
            // Get latest frame
            auto frame = ring_buffer_->get_latest_frame();
            if (frame) {
                // Process frame
                process_frame(frame);
                
                // Get camera feed
                cv::Mat camera_feed = get_camera_feed(frame);

                // Optional: draw detected face ROI for visual debugging
                if (quality_gate_) {
                    FaceROI face = quality_gate_->detect_face(frame);
                    if (face.detected) {
                        cv::rectangle(camera_feed, face.bbox, cv::Scalar(0, 255, 0), 2);
                    }
                }
                
                // Resize camera feed to fit display
                cv::Mat resized_feed;
                cv::resize(camera_feed, resized_feed, cv::Size(640, 480));
                
                // Place camera feed on display
                cv::Rect feed_rect(20, 20, 640, 480);
                resized_feed.copyTo(display(feed_rect));
                
                // Add analysis overlay
                add_analysis_overlay(display, frame);

                // Release frame back to ring buffer to avoid slot timeouts
                ring_buffer_->release_frame(frame);
            }
        } else {
            // Show "Camera Off" message
            cv::putText(display, "Camera Off - Press 'C' to start", 
                       cv::Point(250, 250), cv::FONT_HERSHEY_SIMPLEX, 1.0, 
                       cv::Scalar(0, 0, 255), 2);
        }
        
        // Add control panel
        add_control_panel(display);
        
        // Show display
        cv::imshow("Liveness Viewer", display);
    }
    
    void process_frame(FrameBox* frame) {
        total_frames_++;
        
        // OPTIMIZATION: Run face detection every 3 frames (reduces CPU load by 66%)
        if (total_frames_ % 3 == 0) {
            if (face_detector_) {
                face_detector_->detect(frame);
            }
        } else {
            // Reuse previous face detection if available
            if (last_face_detected_) {
                frame->metadata.face_detected = last_face_detected_;
                frame->metadata.face_x = last_face_x_;
                frame->metadata.face_y = last_face_y_;
                frame->metadata.face_w = last_face_w_;
                frame->metadata.face_h = last_face_h_;
                frame->metadata.face_detection_confidence = last_face_confidence_;
                frame->metadata.landmarks = last_landmarks_;  // CACHE LANDMARKS TOO!
            }
        }
        
        // Cache face detection results (including landmarks!)
        last_face_detected_ = frame->metadata.face_detected;
        last_face_x_ = frame->metadata.face_x;
        last_face_y_ = frame->metadata.face_y;
        last_face_w_ = frame->metadata.face_w;
        last_face_h_ = frame->metadata.face_h;
        last_face_confidence_ = frame->metadata.face_detection_confidence;
        last_landmarks_ = frame->metadata.landmarks;
        
        // Step 2: Quality gate analysis (lightweight)
        bool quality_ok = quality_gate_->process_frame(frame);
        
        // Step 3: Anti-spoofing analysis (only if face detected to save CPU)
        bool anti_spoof_ok = false;
        if (frame->metadata.face_detected) {
            anti_spoof_ok = anti_spoofing_->process_frame(frame);
        }
        
        // Calculate final decision
        bool final_ok = quality_ok && anti_spoof_ok;
        frame->metadata.is_ready_for_processing = final_ok;
        
        // Update statistics
        if (frame->metadata.face_detected) face_detected_++;
        if (final_ok) live_detected_++;
        else spoof_detected_++;
        
        // Print detailed scores every 30 frames
        if (total_frames_ % 30 == 0) {
            print_detailed_scores(frame);
        }
    }
    
    void print_detailed_scores(const FrameBox* frame) {
        std::cout << "\n=== Frame " << total_frames_ << " Analysis ===" << std::endl;
        
        // Quality Gate Results
        std::cout << "Quality Gate: " << (frame->metadata.quality_gate.quality_gate_passed ? "✅ PASSED" : "❌ FAILED") << std::endl;
        std::cout << "  Lighting: " << std::fixed << std::setprecision(3) << frame->metadata.quality_gate.lighting_score << std::endl;
        std::cout << "  Motion: " << frame->metadata.quality_gate.motion_score << std::endl;
        std::cout << "  Positioning: " << frame->metadata.quality_gate.positioning_score << std::endl;
        std::cout << "  Overall Quality: " << frame->metadata.quality_gate.overall_quality_score << std::endl;
        
        // Anti-Spoofing Results
        std::cout << "Anti-Spoofing: " << (frame->metadata.anti_spoofing.is_live ? "✅ PASSED" : "❌ FAILED") << std::endl;
        std::cout << "  Depth Analysis: " << frame->metadata.anti_spoofing.depth_analysis_score << std::endl;
        std::cout << "  IR Texture: " << frame->metadata.anti_spoofing.ir_texture_score << std::endl;
        std::cout << "  Skin Texture: " << frame->metadata.anti_spoofing.skin_texture_score << " ⭐ KEY!" << std::endl;
        std::cout << "  Cross-Modal: " << frame->metadata.anti_spoofing.cross_modal_score << std::endl;
        std::cout << "  Overall Liveness: " << frame->metadata.anti_spoofing.overall_liveness_score << std::endl;
        std::cout << "  Confidence: " << frame->metadata.anti_spoofing.confidence << std::endl;
        
        // Final Decision
        std::cout << "FINAL DECISION: " << (frame->metadata.is_ready_for_processing ? "✅ LIVE HUMAN" : "❌ SPOOF ATTACK") << std::endl;
        if (!frame->metadata.quality_gate.quality_issues.empty()) {
            std::cout << "  Quality Issues: " << frame->metadata.quality_gate.quality_issues << std::endl;
        }
        if (!frame->metadata.anti_spoofing.rejection_reason.empty()) {
            std::cout << "  Rejection Reason: " << frame->metadata.anti_spoofing.rejection_reason << std::endl;
        }
    }
    
    cv::Mat get_camera_feed(FrameBox* frame) {
        cv::Mat feed = cv::Mat::zeros(480, 640, CV_8UC3);
        
        // Try to get color image
        if (!frame->color_data.empty() && frame->color_width > 0 && frame->color_height > 0) {
            cv::Mat color_image(frame->color_height, frame->color_width, CV_8UC3, 
                               const_cast<uint8_t*>(frame->color_data.data()));
            cv::cvtColor(color_image, feed, cv::COLOR_RGB2BGR);
        }
        // Fallback to depth visualization
        else if (!frame->depth_data.empty() && frame->depth_width > 0 && frame->depth_height > 0) {
            cv::Mat depth_image(frame->depth_height, frame->depth_width, CV_16UC1, 
                               const_cast<uint16_t*>(frame->depth_data.data()));
            cv::Mat depth_vis;
            depth_image.convertTo(depth_vis, CV_8U, 255.0 / 5000.0);
            cv::cvtColor(depth_vis, feed, cv::COLOR_GRAY2BGR);
        }
        
        return feed;
    }
    
    void add_analysis_overlay(cv::Mat& display, FrameBox* frame) {
        // Create analysis panel
        cv::Mat analysis_panel = cv::Mat::zeros(480, 320, CV_8UC3);
        
        // Title
        cv::putText(analysis_panel, "LIVENESS ANALYSIS", cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        
        // Quality Gate Results
        cv::putText(analysis_panel, "Quality Gate:", cv::Point(10, 60), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        cv::Scalar quality_color = frame->metadata.quality_gate.quality_gate_passed ? 
                                  cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(analysis_panel, frame->metadata.quality_gate.quality_gate_passed ? "PASS" : "FAIL", 
                   cv::Point(120, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, quality_color, 2);
        
        cv::putText(analysis_panel, "Lighting: " + std::to_string(frame->metadata.quality_gate.lighting_score).substr(0, 4), 
                   cv::Point(10, 85), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        cv::putText(analysis_panel, "Motion: " + std::to_string(frame->metadata.quality_gate.motion_score).substr(0, 4), 
                   cv::Point(10, 105), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        cv::putText(analysis_panel, "Position: " + std::to_string(frame->metadata.quality_gate.positioning_score).substr(0, 4), 
                   cv::Point(10, 125), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        
        // Anti-Spoofing Results
        cv::putText(analysis_panel, "Anti-Spoofing:", cv::Point(10, 160), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        cv::Scalar liveness_color = frame->metadata.anti_spoofing.is_live ? 
                                   cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(analysis_panel, frame->metadata.anti_spoofing.is_live ? "LIVE" : "FAKE", 
                   cv::Point(120, 160), cv::FONT_HERSHEY_SIMPLEX, 0.5, liveness_color, 2);
        
        cv::putText(analysis_panel, "Depth: " + std::to_string(frame->metadata.anti_spoofing.depth_analysis_score).substr(0, 4), 
                   cv::Point(10, 185), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        cv::putText(analysis_panel, "IR Texture: " + std::to_string(frame->metadata.anti_spoofing.ir_texture_score).substr(0, 4), 
                   cv::Point(10, 205), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        cv::putText(analysis_panel, "Cross-Modal: " + std::to_string(frame->metadata.anti_spoofing.cross_modal_score).substr(0, 4), 
                   cv::Point(10, 225), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        cv::putText(analysis_panel, "Liveness: " + std::to_string(frame->metadata.anti_spoofing.overall_liveness_score).substr(0, 4), 
                   cv::Point(10, 245), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        cv::putText(analysis_panel, "Confidence: " + std::to_string(frame->metadata.anti_spoofing.confidence).substr(0, 4), 
                   cv::Point(10, 265), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(200, 200, 200), 1);
        
        // Final Decision
        cv::putText(analysis_panel, "Final Decision:", cv::Point(10, 300), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        cv::Scalar final_color = frame->metadata.is_ready_for_processing ? 
                                cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(analysis_panel, frame->metadata.is_ready_for_processing ? "ACCEPT" : "REJECT", 
                   cv::Point(120, 300), cv::FONT_HERSHEY_SIMPLEX, 0.5, final_color, 2);
        
        // Rejection reason
        if (!frame->metadata.anti_spoofing.rejection_reason.empty()) {
            cv::putText(analysis_panel, "Reason: " + frame->metadata.anti_spoofing.rejection_reason, 
                       cv::Point(10, 325), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
        }
        
        // Place analysis panel on display
        cv::Rect analysis_rect(680, 20, 320, 480);
        analysis_panel.copyTo(display(analysis_rect));
    }
    
    void add_control_panel(cv::Mat& display) {
        // Statistics panel
        cv::Mat stats_panel = cv::Mat::zeros(80, 1000, CV_8UC3);
        
        // Statistics
        double face_rate = total_frames_ > 0 ? (double)face_detected_ / total_frames_ * 100.0 : 0.0;
        double live_rate = total_frames_ > 0 ? (double)live_detected_ / total_frames_ * 100.0 : 0.0;
        double spoof_rate = total_frames_ > 0 ? (double)spoof_detected_ / total_frames_ * 100.0 : 0.0;
        
        cv::putText(stats_panel, "Frames: " + std::to_string(total_frames_), cv::Point(20, 25), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(stats_panel, "Face: " + std::to_string(face_rate).substr(0, 5) + "%", cv::Point(200, 25), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(stats_panel, "Live: " + std::to_string(live_rate).substr(0, 5) + "%", cv::Point(400, 25), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        cv::putText(stats_panel, "Spoof: " + std::to_string(spoof_rate).substr(0, 5) + "%", cv::Point(600, 25), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        // Controls
        cv::putText(stats_panel, "Controls: C=Camera, D=Debug, R=Reset, Q=Quit", cv::Point(20, 55), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
        
        // Place stats panel on display
        cv::Rect stats_rect(0, 520, 1000, 80);
        stats_panel.copyTo(display(stats_rect));
    }
    
    void toggle_camera() {
        if (camera_running_) {
            stop_camera();
        } else {
            start_camera();
        }
    }
    
    void start_camera() {
        if (camera_running_) return;
        
        try {
            // Initialize components
            CameraConfig camera_config;
            camera_config.align_to_color = true;  // CRITICAL: Required for accurate ROI mapping
            ring_buffer_ = std::make_unique<DynamicRingBuffer>(32, 5ULL * 1024 * 1024 * 1024);
            producer_ = std::make_unique<Producer>(camera_config, ring_buffer_.get());
            
            // Initialize face detector (MediaPipe GPU or OpenCV DNN fallback)
            face_detector_ = create_face_detector();
            if (!face_detector_) {
                std::cerr << "❌ No face detector available!" << std::endl;
                ring_buffer_.reset();
                producer_.reset();
                return;
            }
            
            quality_gate_ = std::make_unique<QualityGate>(config_);
            anti_spoofing_ = std::make_unique<AntiSpoofingDetector>(config_);
            
            if (producer_->start()) {
                camera_running_ = true;
                std::cout << "✅ Camera started with " << face_detector_->name() << " face detector!" << std::endl;
            } else {
                std::cout << "❌ Failed to start camera" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "❌ Camera error: " << e.what() << std::endl;
        }
    }
    
    void stop_camera() {
        if (!camera_running_) return;
        
        if (producer_) {
            producer_->stop();
        }
        
        producer_.reset();
        ring_buffer_.reset();
        face_detector_.reset();
        quality_gate_.reset();
        anti_spoofing_.reset();
        
        camera_running_ = false;
        std::cout << "🛑 Camera stopped" << std::endl;
    }
    
    void reset_statistics() {
        total_frames_ = 0;
        face_detected_ = 0;
        live_detected_ = 0;
        spoof_detected_ = 0;
        std::cout << "📊 Statistics reset" << std::endl;
    }
    
    void cleanup() {
        stop_camera();
        cv::destroyAllWindows();
    }
};

int main() {
    SimpleLivenessViewer viewer;
    viewer.run();
    return 0;
}
