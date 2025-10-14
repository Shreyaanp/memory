#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "AntiSpoofing.hpp"

using namespace mdai;

class VisualFaceDetectionTest {
private:
    struct TestParameters {
        int test_duration_seconds = 10;
        int display_width = 1280;
        int display_height = 720;
        
        // Quality Gate Thresholds (for human faces)
        float min_lighting_score = 0.4f;
        float min_motion_score = 0.3f;
        float min_positioning_score = 0.5f;
        float min_synchronization_score = 0.6f;
        float min_stability_score = 0.4f;
        float min_overall_quality = 0.5f;
        
        // Anti-Spoofing Thresholds (for human faces)
        float min_depth_analysis_score = 0.4f;
        float min_ir_texture_score = 0.3f;
        float min_temporal_consistency_score = 0.3f;
        float min_cross_modal_score = 0.4f;
        float min_overall_liveness = 0.5f;
        float min_confidence = 0.6f;
    };
    
    struct TestResults {
        int total_frames = 0;
        int quality_passed = 0;
        int anti_spoofing_passed = 0;
        int final_passed = 0;
        double total_processing_time = 0.0;
        float current_quality_score = 0.0f;
        float current_liveness_score = 0.0f;
        float current_confidence = 0.0f;
        std::string current_status = "Initializing...";
        std::string rejection_reason = "";
    };
    
    TestParameters params_;
    TestResults results_;
    AntiSpoofingConfig config_;
    
public:
    VisualFaceDetectionTest() {
        setup_test_parameters();
    }
    
    void setup_test_parameters() {
        // Configure for human face detection (more lenient)
        config_.min_lighting_score = params_.min_lighting_score;
        config_.min_motion_score = params_.min_motion_score;
        config_.min_positioning_score = params_.min_positioning_score;
        config_.min_synchronization_score = params_.min_synchronization_score;
        config_.min_stability_score = params_.min_stability_score;
        config_.min_overall_quality = params_.min_overall_quality;
        
        config_.min_depth_analysis_score = params_.min_depth_analysis_score;
        config_.min_ir_texture_score = params_.min_ir_texture_score;
        config_.min_temporal_consistency_score = params_.min_temporal_consistency_score;
        config_.min_cross_modal_score = params_.min_cross_modal_score;
        config_.min_overall_liveness = params_.min_overall_liveness;
        config_.min_confidence = params_.min_confidence;
    }
    
    void run_test() {
        std::cout << "🎥 VISUAL FACE DETECTION TEST" << std::endl;
        std::cout << "=============================" << std::endl;
        std::cout << "Duration: " << params_.test_duration_seconds << " seconds" << std::endl;
        std::cout << "Display: " << params_.display_width << "x" << params_.display_height << std::endl;
        std::cout << "Target: Detect and validate human faces with visual feedback" << std::endl;
        
        std::cout << "\n⚠️  INSTRUCTIONS:" << std::endl;
        std::cout << "1. Position your face in front of the camera" << std::endl;
        std::cout << "2. Ensure good lighting (not too bright/dark)" << std::endl;
        std::cout << "3. Keep relatively still during the test" << std::endl;
        std::cout << "4. A window will show the camera feed and analysis" << std::endl;
        std::cout << "5. Test will run for " << params_.test_duration_seconds << " seconds" << std::endl;
        std::cout << "\nPress Enter to start the test..." << std::endl;
        std::cin.get();
        
        // Initialize components
        CameraConfig camera_config;
        camera_config.align_to_color = true;  // Required for accurate ROI mapping
        DynamicRingBuffer ring_buffer(32, 5ULL * 1024 * 1024 * 1024); // 5GB limit
        Producer producer(camera_config, &ring_buffer);
        QualityGate quality_gate(config_);
        AntiSpoofingDetector anti_spoofing(config_);
        
        try {
            std::cout << "\n🚀 Starting camera..." << std::endl;
            if (!producer.start()) {
                std::cerr << "❌ Failed to start producer!" << std::endl;
                return;
            }
            
            std::cout << "✅ Camera started successfully!" << std::endl;
            
            // Set recording active
            ring_buffer.set_recording_active(true);
            std::cout << "📹 Recording started..." << std::endl;
            
            // Run visual test
            run_visual_detection_loop(ring_buffer, quality_gate, anti_spoofing);
            
        } catch (const std::exception& e) {
            std::cerr << "❌ Exception occurred: " << e.what() << std::endl;
        }
        
        // Cleanup
        ring_buffer.set_recording_active(false);
        producer.stop();
        ring_buffer.clear();
        
        // Print final results
        print_final_results();
    }
    
private:
    void run_visual_detection_loop(DynamicRingBuffer& ring_buffer, 
                                  QualityGate& quality_gate, 
                                  AntiSpoofingDetector& anti_spoofing) {
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Create OpenCV windows
        cv::namedWindow("Face Detection Test", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Analysis Results", cv::WINDOW_AUTOSIZE);
        
        std::cout << "\n🎥 Visual windows opened. Press 'q' to quit early." << std::endl;
        
        while (true) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();
            
            if (elapsed_seconds >= params_.test_duration_seconds) {
                break;
            }
            
            // Process frame
            auto frame = ring_buffer.get_latest_frame();
            if (frame) {
                results_.total_frames++;
                
                auto process_start = std::chrono::high_resolution_clock::now();
                
                // Run quality gate
                quality_gate.process_frame(frame);
                bool quality_ok = frame->metadata.quality_gate.quality_gate_passed;
                if (quality_ok) results_.quality_passed++;
                
                // Run anti-spoofing if quality gate passed
                bool anti_spoof_ok = false;
                if (quality_ok) {
                    anti_spoofing.process_frame(frame);
                    anti_spoof_ok = frame->metadata.anti_spoofing.is_live && 
                                   frame->metadata.anti_spoofing.confidence >= config_.min_confidence;
                    if (anti_spoof_ok) results_.anti_spoofing_passed++;
                }
                
                // Final decision
                bool final_ok = quality_ok && anti_spoof_ok;
                if (final_ok) results_.final_passed++;
                
                auto process_end = std::chrono::high_resolution_clock::now();
                double processing_time = std::chrono::duration<double, std::milli>(process_end - process_start).count();
                results_.total_processing_time += processing_time;
                
                // Update current results
                results_.current_quality_score = frame->metadata.quality_gate.overall_quality_score;
                results_.current_liveness_score = frame->metadata.anti_spoofing.overall_liveness_score;
                results_.current_confidence = frame->metadata.anti_spoofing.confidence;
                
                // Update status
                if (final_ok) {
                    results_.current_status = "✅ HUMAN DETECTED";
                    results_.rejection_reason = "";
                } else if (anti_spoof_ok) {
                    results_.current_status = "⚠️ LIVENESS CHECK";
                    results_.rejection_reason = "";
                } else if (quality_ok) {
                    results_.current_status = "🔍 QUALITY CHECK";
                    results_.rejection_reason = frame->metadata.anti_spoofing.rejection_reason;
                } else {
                    results_.current_status = "❌ QUALITY FAILED";
                    results_.rejection_reason = frame->metadata.quality_gate.quality_issues;
                }
                
                // Create visual displays
                create_camera_display(std::shared_ptr<FrameBox>(frame, [](FrameBox*){}), elapsed_seconds);
                create_analysis_display(elapsed_seconds);
                
                // Release frame
                ring_buffer.release_frame(frame);
            }
            
            // Check for quit key
            char key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q') {
                std::cout << "\n⏹️ Test stopped by user." << std::endl;
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        }
        
        cv::destroyAllWindows();
    }
    
    void create_camera_display(std::shared_ptr<FrameBox> frame, double elapsed_time) {
        // Get color image
        cv::Mat color_image;
        if (!frame->color_data.empty() && frame->color_width > 0 && frame->color_height > 0) {
            color_image = cv::Mat(frame->color_height, frame->color_width, CV_8UC3, frame->color_data.data());
            cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
        } else {
            // Create a black image if no color data
            color_image = cv::Mat::zeros(480, 640, CV_8UC3);
        }
        
        // Resize for display
        cv::Mat display_image;
        cv::resize(color_image, display_image, cv::Size(params_.display_width, params_.display_height));
        
        // Add overlay information
        add_overlay_info(display_image, elapsed_time);
        
        // Show the image
        cv::imshow("Face Detection Test", display_image);
    }
    
    void add_overlay_info(cv::Mat& image, double elapsed_time) {
        // Define colors
        cv::Scalar green(0, 255, 0);
        cv::Scalar red(0, 0, 255);
        cv::Scalar yellow(0, 255, 255);
        cv::Scalar white(255, 255, 255);
        cv::Scalar black(0, 0, 0);
        
        // Choose color based on status
        cv::Scalar status_color = green;
        if (results_.current_status.find("❌") != std::string::npos) {
            status_color = red;
        } else if (results_.current_status.find("⚠️") != std::string::npos) {
            status_color = yellow;
        }
        
        // Add status text
        cv::putText(image, results_.current_status, cv::Point(20, 40), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.5, status_color, 3);
        
        // Add time remaining
        double time_remaining = params_.test_duration_seconds - elapsed_time;
        std::string time_text = "Time: " + std::to_string((int)time_remaining) + "s";
        cv::putText(image, time_text, cv::Point(20, 80), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, white, 2);
        
        // Add frame count
        std::string frame_text = "Frames: " + std::to_string(results_.total_frames);
        cv::putText(image, frame_text, cv::Point(20, 120), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, white, 2);
        
        // Add scores
        std::string quality_text = "Quality: " + std::to_string(results_.current_quality_score).substr(0, 4);
        cv::putText(image, quality_text, cv::Point(20, 160), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, white, 2);
        
        std::string liveness_text = "Liveness: " + std::to_string(results_.current_liveness_score).substr(0, 4);
        cv::putText(image, liveness_text, cv::Point(20, 190), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, white, 2);
        
        std::string confidence_text = "Confidence: " + std::to_string(results_.current_confidence).substr(0, 4);
        cv::putText(image, confidence_text, cv::Point(20, 220), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.8, white, 2);
        
        // Add rejection reason if any
        if (!results_.rejection_reason.empty()) {
            cv::putText(image, "Reason: " + results_.rejection_reason, cv::Point(20, 260), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, red, 2);
        }
        
        // Add progress bar
        double progress = elapsed_time / params_.test_duration_seconds;
        int bar_width = 400;
        int bar_height = 20;
        int bar_x = 20;
        int bar_y = 300;
        
        // Background
        cv::rectangle(image, cv::Point(bar_x, bar_y), 
                     cv::Point(bar_x + bar_width, bar_y + bar_height), black, -1);
        
        // Progress
        cv::rectangle(image, cv::Point(bar_x, bar_y), 
                     cv::Point(bar_x + (int)(bar_width * progress), bar_y + bar_height), green, -1);
        
        // Border
        cv::rectangle(image, cv::Point(bar_x, bar_y), 
                     cv::Point(bar_x + bar_width, bar_y + bar_height), white, 2);
    }
    
    void create_analysis_display(double elapsed_time) {
        // Create analysis display
        cv::Mat analysis_image = cv::Mat::zeros(400, 600, CV_8UC3);
        
        // Define colors
        cv::Scalar green(0, 255, 0);
        cv::Scalar red(0, 0, 255);
        cv::Scalar yellow(0, 255, 255);
        cv::Scalar white(255, 255, 255);
        cv::Scalar black(0, 0, 0);
        
        // Title
        cv::putText(analysis_image, "ANALYSIS RESULTS", cv::Point(20, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, white, 2);
        
        // Statistics
        double quality_rate = (double)results_.quality_passed / results_.total_frames * 100.0;
        double anti_spoof_rate = (double)results_.anti_spoofing_passed / results_.total_frames * 100.0;
        double final_rate = (double)results_.final_passed / results_.total_frames * 100.0;
        double avg_processing_time = results_.total_processing_time / results_.total_frames;
        
        // Quality Gate Results
        cv::putText(analysis_image, "Quality Gate:", cv::Point(20, 70), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2);
        cv::putText(analysis_image, std::to_string(results_.quality_passed) + "/" + std::to_string(results_.total_frames) + 
                   " (" + std::to_string((int)quality_rate) + "%)", cv::Point(20, 100), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, quality_rate > 80 ? green : red, 2);
        
        // Anti-Spoofing Results
        cv::putText(analysis_image, "Anti-Spoofing:", cv::Point(20, 140), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2);
        cv::putText(analysis_image, std::to_string(results_.anti_spoofing_passed) + "/" + std::to_string(results_.total_frames) + 
                   " (" + std::to_string((int)anti_spoof_rate) + "%)", cv::Point(20, 170), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, anti_spoof_rate > 80 ? green : red, 2);
        
        // Final Results
        cv::putText(analysis_image, "Final Decision:", cv::Point(20, 210), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2);
        cv::putText(analysis_image, std::to_string(results_.final_passed) + "/" + std::to_string(results_.total_frames) + 
                   " (" + std::to_string((int)final_rate) + "%)", cv::Point(20, 240), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, final_rate > 80 ? green : red, 2);
        
        // Processing Time
        cv::putText(analysis_image, "Avg Processing Time:", cv::Point(20, 280), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2);
        cv::putText(analysis_image, std::to_string((int)avg_processing_time) + "ms", cv::Point(20, 310), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, avg_processing_time < 100 ? green : red, 2);
        
        // Current Scores
        cv::putText(analysis_image, "Current Scores:", cv::Point(300, 70), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.7, white, 2);
        cv::putText(analysis_image, "Quality: " + std::to_string(results_.current_quality_score).substr(0, 4), 
                   cv::Point(300, 100), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 2);
        cv::putText(analysis_image, "Liveness: " + std::to_string(results_.current_liveness_score).substr(0, 4), 
                   cv::Point(300, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 2);
        cv::putText(analysis_image, "Confidence: " + std::to_string(results_.current_confidence).substr(0, 4), 
                   cv::Point(300, 160), cv::FONT_HERSHEY_SIMPLEX, 0.6, white, 2);
        
        // Show the analysis image
        cv::imshow("Analysis Results", analysis_image);
    }
    
    void print_final_results() {
        std::cout << "\n📊 FINAL TEST RESULTS" << std::endl;
        std::cout << "=====================" << std::endl;
        
        // Basic statistics
        std::cout << "Total Frames Processed: " << results_.total_frames << std::endl;
        std::cout << "Quality Gate Passed: " << results_.quality_passed << " (" 
                  << std::fixed << std::setprecision(1) 
                  << (100.0 * results_.quality_passed / results_.total_frames) << "%)" << std::endl;
        std::cout << "Anti-Spoofing Passed: " << results_.anti_spoofing_passed << " (" 
                  << (100.0 * results_.anti_spoofing_passed / results_.total_frames) << "%)" << std::endl;
        std::cout << "Final Decision Passed: " << results_.final_passed << " (" 
                  << (100.0 * results_.final_passed / results_.total_frames) << "%)" << std::endl;
        std::cout << "Average Processing Time: " << (results_.total_processing_time / results_.total_frames) << "ms" << std::endl;
        
        // Performance analysis
        std::cout << "\n🎯 PERFORMANCE ANALYSIS:" << std::endl;
        if (results_.total_processing_time / results_.total_frames < 100.0) {
            std::cout << "  ✅ Processing time is excellent (<100ms)" << std::endl;
        } else {
            std::cout << "  ⚠️  Processing time exceeds 100ms threshold" << std::endl;
        }
        
        if (results_.final_passed > 0) {
            std::cout << "  ✅ System successfully detected human faces" << std::endl;
        } else {
            std::cout << "  ❌ System failed to detect any human faces" << std::endl;
        }
        
        std::cout << "\n✅ Visual test completed!" << std::endl;
    }
};

int main() {
    std::cout << "🎥 Visual Face Detection Test" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "This test will show real-time camera feed and analysis results." << std::endl;
    std::cout << "\n⚠️  IMPORTANT: Ensure you have a RealSense D435 camera connected!" << std::endl;
    std::cout << "⚠️  IMPORTANT: OpenCV is required for visual display!" << std::endl;
    
    VisualFaceDetectionTest test;
    test.run_test();
    
    return 0;
}
