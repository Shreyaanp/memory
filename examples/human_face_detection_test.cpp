#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <string>
#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "AntiSpoofing.hpp"
#include "FaceDetector.hpp"

using namespace mdai;

class HumanFaceDetectionTest {
private:
    struct TestParameters {
        // Test Configuration
        int test_duration_seconds = 10;
        int display_update_interval = 1; // Update display every 1 second
        
        // Quality Gate Thresholds (for human faces)
        float min_lighting_score = 0.4f;      // Lower for normal lighting
        float min_motion_score = 0.3f;        // Lower for natural movement
        float min_positioning_score = 0.5f;   // Lower for various face positions
        float min_synchronization_score = 0.6f;
        float min_stability_score = 0.4f;
        float min_overall_quality = 0.5f;     // Lower overall quality threshold
        
        // Anti-Spoofing Thresholds (for human faces)
        float min_depth_analysis_score = 0.4f;    // Lower for real faces
        float min_ir_texture_score = 0.3f;        // Lower for skin texture
        float min_temporal_consistency_score = 0.3f;
        float min_cross_modal_score = 0.4f;       // Lower cross-modal threshold
        float min_overall_liveness = 0.5f;        // Lower liveness threshold
        float min_confidence = 0.6f;              // Lower confidence threshold
        
        // Display Settings
        bool show_detailed_scores = true;
        bool show_frame_analysis = true;
        bool show_rejection_reasons = true;
    };
    
    struct TestResults {
        int total_frames = 0;
        int quality_passed = 0;
        int anti_spoofing_passed = 0;
        int final_passed = 0;
        double total_processing_time = 0.0;
        std::vector<std::string> rejection_reasons;
        std::vector<float> quality_scores;
        std::vector<float> liveness_scores;
        std::vector<float> confidence_scores;
    };
    
    TestParameters params_;
    TestResults results_;
    AntiSpoofingConfig config_;
    
public:
    HumanFaceDetectionTest() {
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
        std::cout << "🧪 HUMAN FACE DETECTION TEST" << std::endl;
        std::cout << "🔧 Starting test initialization..." << std::endl;
        std::cout << "============================" << std::endl;
        std::cout << "Duration: " << params_.test_duration_seconds << " seconds" << std::endl;
        std::cout << "Target: Detect and validate human faces" << std::endl;
        std::cout << "\n📋 TEST PARAMETERS:" << std::endl;
        print_test_parameters();
        
        std::cout << "\n⚠️  INSTRUCTIONS:" << std::endl;
        std::cout << "1. Position your face in front of the camera" << std::endl;
        std::cout << "2. Ensure good lighting (not too bright/dark)" << std::endl;
        std::cout << "3. Keep relatively still during the test" << std::endl;
        std::cout << "4. Test will run for " << params_.test_duration_seconds << " seconds" << std::endl;
        std::cout << "\nPress Enter to start the test..." << std::endl;
        // std::cin.get(); // Skip user input for automated testing
        
        // Initialize components
        CameraConfig camera_config;
        camera_config.align_to_color = true;  // Required for accurate ROI mapping
        DynamicRingBuffer ring_buffer(32, 5ULL * 1024 * 1024 * 1024); // 5GB limit
        Producer producer(camera_config, &ring_buffer);
        
        // Initialize face detector (MediaPipe or OpenCV DNN fallback)
        std::cout << "🔧 Creating face detector..." << std::endl;
        auto face_detector = create_face_detector();
        if (!face_detector) {
            std::cerr << "❌ No face detector available!" << std::endl;
            return;
        }
        std::cout << "✓ Face detector initialized: " << face_detector->name() << std::endl;
        
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
            
            // Clear screen for real-time display
            system("clear");
            
            // Run test
            run_detection_loop(ring_buffer, quality_gate, anti_spoofing, face_detector.get());
            
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
    void print_test_parameters() {
        std::cout << "  Quality Gate Thresholds:" << std::endl;
        std::cout << "    Lighting: " << std::fixed << std::setprecision(2) << config_.min_lighting_score << std::endl;
        std::cout << "    Motion: " << config_.min_motion_score << std::endl;
        std::cout << "    Positioning: " << config_.min_positioning_score << std::endl;
        std::cout << "    Overall Quality: " << config_.min_overall_quality << std::endl;
        std::cout << "  Anti-Spoofing Thresholds:" << std::endl;
        std::cout << "    Depth Analysis: " << config_.min_depth_analysis_score << std::endl;
        std::cout << "    IR Texture: " << config_.min_ir_texture_score << std::endl;
        std::cout << "    Cross-Modal: " << config_.min_cross_modal_score << std::endl;
        std::cout << "    Overall Liveness: " << config_.min_overall_liveness << std::endl;
        std::cout << "    Confidence: " << config_.min_confidence << std::endl;
    }
    
    void run_detection_loop(DynamicRingBuffer& ring_buffer, 
                           QualityGate& quality_gate, 
                           AntiSpoofingDetector& anti_spoofing,
                           FaceDetector* face_detector) {
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_display_update = start_time;
        int frame_count = 0;
        
        std::cout << "🔍 REAL-TIME FACE DETECTION" << std::endl;
        std::cout << "===========================" << std::endl;
        std::cout << "Time | Frames | Quality | Anti-Spoof | Final | Avg Time | Status" << std::endl;
        std::cout << "-----|--------|---------|------------|-------|----------|--------" << std::endl;
        
        while (true) {
            auto current_time = std::chrono::high_resolution_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();
            
            if (elapsed_seconds >= params_.test_duration_seconds) {
                break;
            }
            
            // Process frame
            auto frame = ring_buffer.get_latest_frame();
            if (frame) {
                frame_count++;
                results_.total_frames++;
                
                auto process_start = std::chrono::high_resolution_clock::now();
                
                // STEP 1: Face detection (MediaPipe or OpenCV DNN)
                if (face_detector) {
                    face_detector->detect(frame);
                    // Debug: Check if face was detected
                    if (frame->metadata.face_detected) {
                        std::cout << "🔍 Face detected! Confidence: " << frame->metadata.face_detection_confidence << std::endl;
                    }
                }
                
                // STEP 2: Run quality gate
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
                
                // Store scores for analysis
                results_.quality_scores.push_back(frame->metadata.quality_gate.overall_quality_score);
                results_.liveness_scores.push_back(frame->metadata.anti_spoofing.overall_liveness_score);
                results_.confidence_scores.push_back(frame->metadata.anti_spoofing.confidence);
                
                // Store rejection reasons
                if (!quality_ok) {
                    results_.rejection_reasons.push_back("Quality: " + frame->metadata.quality_gate.quality_issues);
                } else if (!anti_spoof_ok) {
                    results_.rejection_reasons.push_back("Anti-spoof: " + frame->metadata.anti_spoofing.rejection_reason);
                }
                
                // Update display every second
                if (std::chrono::duration<double>(current_time - last_display_update).count() >= params_.display_update_interval) {
                    update_display(elapsed_seconds, frame_count, quality_ok, anti_spoof_ok, final_ok, processing_time);
                    last_display_update = current_time;
                }
                
                // Release frame
                ring_buffer.release_frame(frame);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        }
    }
    
    void update_display(double elapsed_time, int frame_count, bool quality_ok, bool anti_spoof_ok, bool final_ok, double processing_time) {
        // Clear current line and move cursor up
        std::cout << "\r\033[K";
        
        // Calculate averages
        double avg_processing_time = results_.total_processing_time / results_.total_frames;
        double quality_rate = (double)results_.quality_passed / results_.total_frames * 100.0;
        double anti_spoof_rate = (double)results_.anti_spoofing_passed / results_.total_frames * 100.0;
        double final_rate = (double)results_.final_passed / results_.total_frames * 100.0;
        
        // Status indicator
        std::string status = "🔍 Scanning";
        if (final_ok) {
            status = "✅ Human Detected";
        } else if (anti_spoof_ok) {
            status = "⚠️  Liveness Check";
        } else if (quality_ok) {
            status = "🔍 Quality Check";
        } else {
            status = "❌ Quality Failed";
        }
        
        // Display real-time info
        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(4) << elapsed_time << "s | "
                  << std::setw(6) << frame_count << " | "
                  << std::setw(7) << (quality_ok ? "PASS" : "FAIL") << " | "
                  << std::setw(10) << (anti_spoof_ok ? "PASS" : "FAIL") << " | "
                  << std::setw(5) << (final_ok ? "PASS" : "FAIL") << " | "
                  << std::setw(8) << std::setprecision(1) << avg_processing_time << "ms | "
                  << status;
        
        std::cout.flush();
    }
    
    void print_final_results() {
        std::cout << "\n\n📊 FINAL TEST RESULTS" << std::endl;
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
        
        // Score analysis
        if (!results_.quality_scores.empty()) {
            float avg_quality = 0.0f, avg_liveness = 0.0f, avg_confidence = 0.0f;
            for (size_t i = 0; i < results_.quality_scores.size(); ++i) {
                avg_quality += results_.quality_scores[i];
                avg_liveness += results_.liveness_scores[i];
                avg_confidence += results_.confidence_scores[i];
            }
            avg_quality /= results_.quality_scores.size();
            avg_liveness /= results_.liveness_scores.size();
            avg_confidence /= results_.confidence_scores.size();
            
            std::cout << "\n📈 AVERAGE SCORES:" << std::endl;
            std::cout << "  Quality Score: " << std::fixed << std::setprecision(3) << avg_quality << std::endl;
            std::cout << "  Liveness Score: " << avg_liveness << std::endl;
            std::cout << "  Confidence Score: " << avg_confidence << std::endl;
        }
        
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
        
        // Recommendations
        std::cout << "\n💡 RECOMMENDATIONS:" << std::endl;
        if (results_.quality_passed < results_.total_frames * 0.5) {
            std::cout << "  - Quality gate is too strict, consider lowering thresholds" << std::endl;
        }
        if (results_.anti_spoofing_passed < results_.quality_passed * 0.5) {
            std::cout << "  - Anti-spoofing is too strict, consider adjusting liveness thresholds" << std::endl;
        }
        if (results_.final_passed == 0) {
            std::cout << "  - No frames passed final validation - check lighting and positioning" << std::endl;
        }
        
        // Common rejection reasons
        if (!results_.rejection_reasons.empty()) {
            std::cout << "\n🔍 COMMON REJECTION REASONS:" << std::endl;
            std::map<std::string, int> reason_counts;
            for (const auto& reason : results_.rejection_reasons) {
                reason_counts[reason]++;
            }
            for (const auto& pair : reason_counts) {
                std::cout << "  - " << pair.first << " (" << pair.second << " times)" << std::endl;
            }
        }
        
        std::cout << "\n✅ Test completed!" << std::endl;
    }
};

int main() {
    std::cout << "🚀 Human Face Detection Test" << std::endl;
    std::cout << "============================" << std::endl;
    std::cout << "This test will validate the anti-spoofing system's ability" << std::endl;
    std::cout << "to detect and accept real human faces." << std::endl;
    std::cout << "\n⚠️  IMPORTANT: Ensure you have a RealSense D435 camera connected!" << std::endl;
    
    HumanFaceDetectionTest test;
    test.run_test();
    
    return 0;
}
