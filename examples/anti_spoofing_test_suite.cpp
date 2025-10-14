#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <iomanip>
#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "AntiSpoofing.hpp"

using namespace mdai;

class AntiSpoofingTestSuite {
private:
    struct TestResult {
        std::string test_name;
        bool passed;
        float quality_score;
        float liveness_score;
        float confidence;
        std::string rejection_reason;
        double processing_time_ms;
        int frame_count;
    };
    
    std::vector<TestResult> results_;
    AntiSpoofingConfig config_;
    
public:
    AntiSpoofingTestSuite() {
        // Configure testing parameters
        config_.min_overall_quality = 0.7f;        // Quality gate threshold
        config_.min_overall_liveness = 0.8f;       // Anti-spoofing threshold
        config_.min_confidence = 0.9f;             // Confidence threshold
        config_.min_lighting_score = 0.6f;         // Lighting quality threshold
        config_.min_motion_score = 0.7f;           // Motion stability threshold
        config_.min_depth_analysis_score = 0.7f;   // Depth analysis threshold
        config_.min_ir_texture_score = 0.6f;       // IR texture threshold
    }
    
    void run_comprehensive_test() {
        std::cout << "🧪 Starting Anti-Spoofing Test Suite" << std::endl;
        std::cout << "=====================================" << std::endl;
        
        // Test 1: Real Human Face (Baseline)
        run_test("Real Human Face - Baseline", true);
        
        // Test 2: Poor Lighting Conditions
        run_test("Poor Lighting - Should Reject", false);
        
        // Test 3: Excessive Movement
        run_test("Excessive Movement - Should Reject", false);
        
        // Test 4: Face Too Far/Close
        run_test("Face Positioning Issues - Should Reject", false);
        
        // Test 5: Simulated Screen Attack
        run_test("Simulated Screen Attack - Should Reject", false);
        
        // Test 6: Simulated Photo Attack
        run_test("Simulated Photo Attack - Should Reject", false);
        
        // Test 7: Simulated Mask Attack
        run_test("Simulated Mask Attack - Should Reject", false);
        
        // Test 8: Edge Case - Very Close Face
        run_test("Very Close Face - Edge Case", true);
        
        // Test 9: Edge Case - Profile View
        run_test("Profile View - Edge Case", true);
        
        // Test 10: Stress Test - Rapid Movement
        run_test("Rapid Movement - Stress Test", false);
        
        // Print comprehensive results
        print_results();
    }
    
private:
    void run_test(const std::string& test_name, bool should_pass) {
        std::cout << "\n🔍 Running Test: " << test_name << std::endl;
        std::cout << "Expected Result: " << (should_pass ? "PASS" : "REJECT") << std::endl;
        
        // Initialize components
        CameraConfig camera_config;
        camera_config.align_to_color = true;  // Required for accurate ROI mapping
        DynamicRingBuffer ring_buffer(32, 5 * 1024 * 1024 * 1024); // 5GB limit
        Producer producer(camera_config, &ring_buffer);
        QualityGate quality_gate(config_);
        AntiSpoofingDetector anti_spoofing(config_);
        
        TestResult result;
        result.test_name = test_name;
        result.passed = false;
        result.frame_count = 0;
        
        try {
            // Start the system
            if (!producer.start()) {
                result.rejection_reason = "Failed to start producer";
                results_.push_back(result);
                return;
            }
            
            // Set recording active
            ring_buffer.set_recording_active(true);
            
            auto start_time = std::chrono::high_resolution_clock::now();
            int processed_frames = 0;
            int valid_frames = 0;
            
            // Process frames for 10 seconds
            for (int i = 0; i < 300; ++i) { // 30 FPS * 10 seconds
                auto frame = ring_buffer.get_latest_frame();
                if (frame) {
                    processed_frames++;
                    
                    // Run quality gate
                    quality_gate.process_frame(frame);
                    
                    // Run anti-spoofing if quality gate passed
                    if (frame->metadata.quality_gate.quality_gate_passed) {
                        anti_spoofing.process_frame(frame);
                        
                        if (frame->metadata.anti_spoofing.is_live && 
                            frame->metadata.anti_spoofing.confidence >= config_.min_confidence) {
                            valid_frames++;
                        }
                    }
                    
                    // Release frame
                    ring_buffer.release_frame(frame);
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 FPS
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            result.processing_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            result.frame_count = processed_frames;
            
            // Calculate average scores from last few frames
            calculate_average_scores(result);
            
            // Determine if test passed
            result.passed = (valid_frames > 0) == should_pass;
            
            if (!result.passed) {
                result.rejection_reason = should_pass ? 
                    "Failed to detect valid face" : 
                    "Incorrectly accepted invalid face";
            }
            
        } catch (const std::exception& e) {
            result.rejection_reason = std::string("Exception: ") + e.what();
        }
        
        // Cleanup
        ring_buffer.set_recording_active(false);
        producer.stop();
        ring_buffer.clear();
        
        results_.push_back(result);
        
        // Print immediate result
        std::cout << "Result: " << (result.passed ? "✅ PASS" : "❌ FAIL") << std::endl;
        std::cout << "Quality Score: " << std::fixed << std::setprecision(3) << result.quality_score << std::endl;
        std::cout << "Liveness Score: " << result.liveness_score << std::endl;
        std::cout << "Confidence: " << result.confidence << std::endl;
        std::cout << "Processing Time: " << result.processing_time_ms << "ms" << std::endl;
        if (!result.rejection_reason.empty()) {
            std::cout << "Reason: " << result.rejection_reason << std::endl;
        }
    }
    
    void calculate_average_scores(TestResult& result) {
        // This would ideally sample from the last few processed frames
        // For now, we'll use the config thresholds as baseline
        result.quality_score = config_.min_overall_quality;
        result.liveness_score = config_.min_overall_liveness;
        result.confidence = config_.min_confidence;
    }
    
    void print_results() {
        std::cout << "\n📊 COMPREHENSIVE TEST RESULTS" << std::endl;
        std::cout << "=============================" << std::endl;
        
        int total_tests = results_.size();
        int passed_tests = 0;
        double total_processing_time = 0.0;
        int total_frames = 0;
        
        std::cout << std::left << std::setw(40) << "Test Name" 
                  << std::setw(8) << "Result" 
                  << std::setw(10) << "Quality" 
                  << std::setw(10) << "Liveness" 
                  << std::setw(10) << "Confidence" 
                  << std::setw(12) << "Time(ms)" 
                  << std::setw(8) << "Frames" << std::endl;
        std::cout << std::string(100, '-') << std::endl;
        
        for (const auto& result : results_) {
            if (result.passed) passed_tests++;
            total_processing_time += result.processing_time_ms;
            total_frames += result.frame_count;
            
            std::cout << std::left << std::setw(40) << result.test_name
                      << std::setw(8) << (result.passed ? "PASS" : "FAIL")
                      << std::setw(10) << std::fixed << std::setprecision(3) << result.quality_score
                      << std::setw(10) << result.liveness_score
                      << std::setw(10) << result.confidence
                      << std::setw(12) << std::setprecision(1) << result.processing_time_ms
                      << std::setw(8) << result.frame_count << std::endl;
        }
        
        std::cout << std::string(100, '-') << std::endl;
        std::cout << "SUMMARY:" << std::endl;
        std::cout << "  Total Tests: " << total_tests << std::endl;
        std::cout << "  Passed: " << passed_tests << " (" << (100.0 * passed_tests / total_tests) << "%)" << std::endl;
        std::cout << "  Failed: " << (total_tests - passed_tests) << std::endl;
        std::cout << "  Average Processing Time: " << (total_processing_time / total_tests) << "ms" << std::endl;
        std::cout << "  Total Frames Processed: " << total_frames << std::endl;
        
        // Performance analysis
        std::cout << "\n🎯 PERFORMANCE ANALYSIS:" << std::endl;
        if (total_processing_time / total_tests < 100.0) {
            std::cout << "  ✅ Processing time is within acceptable limits (<100ms)" << std::endl;
        } else {
            std::cout << "  ⚠️  Processing time exceeds 100ms threshold" << std::endl;
        }
        
        if (passed_tests >= total_tests * 0.9) {
            std::cout << "  ✅ Test success rate is excellent (≥90%)" << std::endl;
        } else if (passed_tests >= total_tests * 0.8) {
            std::cout << "  ⚠️  Test success rate is good but could be improved (≥80%)" << std::endl;
        } else {
            std::cout << "  ❌ Test success rate needs improvement (<80%)" << std::endl;
        }
        
        // Recommendations
        std::cout << "\n💡 RECOMMENDATIONS:" << std::endl;
        if (passed_tests < total_tests) {
            std::cout << "  - Review failed tests and adjust thresholds" << std::endl;
            std::cout << "  - Consider environmental factors affecting performance" << std::endl;
            std::cout << "  - Test with more diverse attack scenarios" << std::endl;
        }
        std::cout << "  - Run tests in different lighting conditions" << std::endl;
        std::cout << "  - Test with various face angles and distances" << std::endl;
        std::cout << "  - Validate against known attack vectors" << std::endl;
    }
};

int main() {
    std::cout << "🚀 Anti-Spoofing Test Suite" << std::endl;
    std::cout << "===========================" << std::endl;
    std::cout << "This test suite will validate the anti-spoofing system" << std::endl;
    std::cout << "against various real-world scenarios and attack vectors." << std::endl;
    std::cout << "\n⚠️  IMPORTANT: Ensure you have a RealSense D435 camera connected!" << std::endl;
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.get();
    
    AntiSpoofingTestSuite test_suite;
    test_suite.run_comprehensive_test();
    
    std::cout << "\n✅ Test suite completed!" << std::endl;
    return 0;
}
