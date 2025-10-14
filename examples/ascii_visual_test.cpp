#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <vector>
#include <string>
#include <cstdlib>
#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "AntiSpoofing.hpp"

using namespace mdai;

class ASCIIVisualTest {
private:
    struct TestParameters {
        int test_duration_seconds = 10;
        int display_update_interval = 500; // Update display every 500ms
        
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
        std::vector<float> quality_history;
        std::vector<float> liveness_history;
    };
    
    TestParameters params_;
    TestResults results_;
    AntiSpoofingConfig config_;
    
public:
    ASCIIVisualTest() {
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
        std::cout << "🎥 ASCII VISUAL FACE DETECTION TEST" << std::endl;
        std::cout << "===================================" << std::endl;
        std::cout << "Duration: " << params_.test_duration_seconds << " seconds" << std::endl;
        std::cout << "Display: ASCII terminal visualization" << std::endl;
        std::cout << "Target: Detect and validate human faces with visual feedback" << std::endl;
        
        std::cout << "\n⚠️  INSTRUCTIONS:" << std::endl;
        std::cout << "1. Position your face in front of the camera" << std::endl;
        std::cout << "2. Ensure good lighting (not too bright/dark)" << std::endl;
        std::cout << "3. Keep relatively still during the test" << std::endl;
        std::cout << "4. Terminal will show real-time analysis" << std::endl;
        std::cout << "5. Test will run for " << params_.test_duration_seconds << " seconds" << std::endl;
        std::cout << "\nPress Enter to start the test..." << std::endl;
        std::cin.get();
        
        // Initialize components
        CameraConfig camera_config;
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
            
            // Clear screen and run visual test
            system("clear");
            run_ascii_visual_loop(ring_buffer, quality_gate, anti_spoofing);
            
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
    void run_ascii_visual_loop(DynamicRingBuffer& ring_buffer, 
                              QualityGate& quality_gate, 
                              AntiSpoofingDetector& anti_spoofing) {
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto last_display_update = start_time;
        
        std::cout << "🎥 REAL-TIME FACE DETECTION VISUALIZATION" << std::endl;
        std::cout << "==========================================" << std::endl;
        
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
                
                // Store history for visualization
                results_.quality_history.push_back(results_.current_quality_score);
                results_.liveness_history.push_back(results_.current_liveness_score);
                
                // Keep only last 20 values for display
                if (results_.quality_history.size() > 20) {
                    results_.quality_history.erase(results_.quality_history.begin());
                    results_.liveness_history.erase(results_.liveness_history.begin());
                }
                
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
                
                // Update display every 500ms
                if (std::chrono::duration<double, std::milli>(current_time - last_display_update).count() >= params_.display_update_interval) {
                    update_ascii_display(elapsed_seconds);
                    last_display_update = current_time;
                }
                
                // Release frame
                ring_buffer.release_frame(frame);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
        }
    }
    
    void update_ascii_display(double elapsed_time) {
        // Clear screen and move cursor to top
        std::cout << "\033[2J\033[H";
        
        // Header
        std::cout << "🎥 REAL-TIME FACE DETECTION VISUALIZATION" << std::endl;
        std::cout << "==========================================" << std::endl;
        
        // Time and progress
        double time_remaining = params_.test_duration_seconds - elapsed_time;
        double progress = elapsed_time / params_.test_duration_seconds;
        
        std::cout << "Time: " << std::fixed << std::setprecision(1) << elapsed_time << "s / " 
                  << params_.test_duration_seconds << "s (Remaining: " << time_remaining << "s)" << std::endl;
        
        // Progress bar
        std::cout << "Progress: [";
        int bar_width = 50;
        int filled = (int)(bar_width * progress);
        for (int i = 0; i < bar_width; i++) {
            if (i < filled) {
                std::cout << "█";
            } else {
                std::cout << "░";
            }
        }
        std::cout << "] " << std::setprecision(0) << (progress * 100) << "%" << std::endl;
        
        std::cout << std::endl;
        
        // Current status
        std::cout << "🔍 CURRENT STATUS: " << results_.current_status << std::endl;
        std::cout << "📊 Frames Processed: " << results_.total_frames << std::endl;
        
        // Current scores
        std::cout << "\n📈 CURRENT SCORES:" << std::endl;
        std::cout << "  Quality Score: " << std::fixed << std::setprecision(3) << results_.current_quality_score 
                  << " (Threshold: " << config_.min_overall_quality << ")" << std::endl;
        std::cout << "  Liveness Score: " << results_.current_liveness_score 
                  << " (Threshold: " << config_.min_overall_liveness << ")" << std::endl;
        std::cout << "  Confidence: " << results_.current_confidence 
                  << " (Threshold: " << config_.min_confidence << ")" << std::endl;
        
        // Rejection reason
        if (!results_.rejection_reason.empty()) {
            std::cout << "\n❌ REJECTION REASON: " << results_.rejection_reason << std::endl;
        }
        
        // Statistics
        double quality_rate = (double)results_.quality_passed / results_.total_frames * 100.0;
        double anti_spoof_rate = (double)results_.anti_spoofing_passed / results_.total_frames * 100.0;
        double final_rate = (double)results_.final_passed / results_.total_frames * 100.0;
        double avg_processing_time = results_.total_processing_time / results_.total_frames;
        
        std::cout << "\n📊 STATISTICS:" << std::endl;
        std::cout << "  Quality Gate: " << results_.quality_passed << "/" << results_.total_frames 
                  << " (" << std::setprecision(1) << quality_rate << "%)" << std::endl;
        std::cout << "  Anti-Spoofing: " << results_.anti_spoofing_passed << "/" << results_.total_frames 
                  << " (" << anti_spoof_rate << "%)" << std::endl;
        std::cout << "  Final Decision: " << results_.final_passed << "/" << results_.total_frames 
                  << " (" << final_rate << "%)" << std::endl;
        std::cout << "  Avg Processing Time: " << std::setprecision(1) << avg_processing_time << "ms" << std::endl;
        
        // ASCII visualization of score history
        if (results_.quality_history.size() > 1) {
            std::cout << "\n📈 SCORE HISTORY (Last 20 frames):" << std::endl;
            create_ascii_chart("Quality", results_.quality_history, 0.0f, 1.0f);
            create_ascii_chart("Liveness", results_.liveness_history, 0.0f, 1.0f);
        }
        
        // Face representation
        std::cout << "\n👤 FACE DETECTION STATUS:" << std::endl;
        create_face_visualization();
        
        std::cout.flush();
    }
    
    void create_ascii_chart(const std::string& title, const std::vector<float>& values, float min_val, float max_val) {
        std::cout << "  " << title << ": ";
        
        if (values.empty()) {
            std::cout << "No data" << std::endl;
            return;
        }
        
        // Create a simple ASCII chart
        int chart_width = 30;
        for (size_t i = 0; i < values.size(); i++) {
            float normalized = (values[i] - min_val) / (max_val - min_val);
            int height = (int)(normalized * 5); // 5 levels
            
            char symbol = '░';
            if (height >= 4) symbol = '█';
            else if (height >= 3) symbol = '▓';
            else if (height >= 2) symbol = '▒';
            else if (height >= 1) symbol = '░';
            else symbol = ' ';
            
            std::cout << symbol;
        }
        
        // Show current value
        std::cout << " " << std::fixed << std::setprecision(2) << values.back();
        std::cout << std::endl;
    }
    
    void create_face_visualization() {
        // Simple ASCII face representation
        std::cout << "    ┌─────────────┐" << std::endl;
        std::cout << "    │  ●       ●  │" << std::endl;
        std::cout << "    │             │" << std::endl;
        std::cout << "    │      ●      │" << std::endl;
        std::cout << "    │  ─────────  │" << std::endl;
        std::cout << "    └─────────────┘" << std::endl;
        
        // Status indicator
        if (results_.current_status.find("✅") != std::string::npos) {
            std::cout << "    Status: DETECTED ✅" << std::endl;
        } else if (results_.current_status.find("⚠️") != std::string::npos) {
            std::cout << "    Status: CHECKING ⚠️" << std::endl;
        } else if (results_.current_status.find("🔍") != std::string::npos) {
            std::cout << "    Status: ANALYZING 🔍" << std::endl;
        } else {
            std::cout << "    Status: FAILED ❌" << std::endl;
        }
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
        
        std::cout << "\n✅ ASCII visual test completed!" << std::endl;
    }
};

int main() {
    std::cout << "🎥 ASCII Visual Face Detection Test" << std::endl;
    std::cout << "===================================" << std::endl;
    std::cout << "This test will show real-time analysis using ASCII visualization." << std::endl;
    std::cout << "\n⚠️  IMPORTANT: Ensure you have a RealSense D435 camera connected!" << std::endl;
    
    ASCIIVisualTest test;
    test.run_test();
    
    return 0;
}
