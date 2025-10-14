#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "Producer.hpp"
#include "RingBuffer.hpp"
#include "RealAntiSpoofing.hpp"

using namespace mdai;

class DebugAntiSpoofing {
private:
    std::unique_ptr<Producer> producer_;
    std::unique_ptr<DynamicRingBuffer> ring_buffer_;
    std::unique_ptr<RealAntiSpoofing> anti_spoofing_;
    
    bool camera_running_ = false;
    int total_frames_ = 0;
    int face_detected_ = 0;
    int live_detected_ = 0;
    int spoof_detected_ = 0;
    std::chrono::high_resolution_clock::time_point start_time;
    
public:
    DebugAntiSpoofing() {
        std::cout << "🔍 Debug Anti-Spoofing Test" << std::endl;
        std::cout << "===========================" << std::endl;
        std::cout << "This will test the anti-spoofing system with detailed output." << std::endl;
        std::cout << "Press Ctrl+C to stop." << std::endl;
    }
    
    void run() {
        try {
            // Initialize components
            CameraConfig camera_config;
            ring_buffer_ = std::make_unique<DynamicRingBuffer>(32, 5ULL * 1024 * 1024 * 1024);
            producer_ = std::make_unique<Producer>(camera_config, ring_buffer_.get());
            anti_spoofing_ = std::make_unique<RealAntiSpoofing>();
            
            std::cout << "\n🎥 Starting camera..." << std::endl;
            if (!producer_->start()) {
                std::cout << "❌ Failed to start camera!" << std::endl;
                return;
            }
            
            camera_running_ = true;
            std::cout << "✅ Camera started successfully!" << std::endl;
            std::cout << "\n📊 Processing frames... (Press Ctrl+C to stop)" << std::endl;
            
            start_time = std::chrono::high_resolution_clock::now();
            
            while (camera_running_) {
                // Get latest frame
                auto frame = ring_buffer_->get_latest_frame();
                
                if (frame) {
                    process_frame(frame);
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
                
                // Print progress every 100 frames
                if (total_frames_ % 100 == 0 && total_frames_ > 0) {
                    print_progress();
                }
                
                // Small delay to prevent overwhelming the system
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
        } catch (const std::exception& e) {
            std::cout << "❌ Error: " << e.what() << std::endl;
        }
        
        cleanup();
    }
    
private:
    void process_frame(FrameBox* frame) {
        total_frames_++;
        
        // Process with anti-spoofing system
        auto result = anti_spoofing_->process_frame(frame);
        
        // Update statistics
        if (result.detected_face.is_valid) face_detected_++;
        if (result.is_live) live_detected_++;
        else spoof_detected_++;
        
        // Print detailed results every 30 frames
        if (total_frames_ % 30 == 0) {
            print_detailed_results(result);
        }
    }
    
    void print_detailed_results(const AntiSpoofingResult& result) {
        std::cout << "\n=== Frame " << total_frames_ << " Analysis ===" << std::endl;
        
        // Face Detection
        std::cout << "Face Detection: " << (result.detected_face.is_valid ? "✅ DETECTED" : "❌ NOT DETECTED") << std::endl;
        if (result.detected_face.is_valid) {
            std::cout << "  Confidence: " << std::fixed << std::setprecision(3) << result.detected_face.confidence << std::endl;
            std::cout << "  BBox: [" << result.detected_face.bbox.x << ", " << result.detected_face.bbox.y 
                      << ", " << result.detected_face.bbox.width << ", " << result.detected_face.bbox.height << "]" << std::endl;
            std::cout << "  Landmarks: " << result.detected_face.landmarks.size() << " points" << std::endl;
        }
        
        // Individual scores
        std::cout << "Scores:" << std::endl;
        std::cout << "  Face Detection: " << std::fixed << std::setprecision(3) << result.face_detection_score << std::endl;
        std::cout << "  Temporal Liveness: " << result.temporal_liveness_score << std::endl;
        std::cout << "  3D Geometry: " << result.geometric_3d_score << std::endl;
        std::cout << "  Spectral Analysis: " << result.spectral_analysis_score << std::endl;
        
        // Temporal Analysis
        std::cout << "Temporal Analysis:" << std::endl;
        std::cout << "  Micro-motion: " << std::fixed << std::setprecision(3) << result.temporal_data.micro_motion_score << std::endl;
        std::cout << "  Breathing: " << result.temporal_data.breathing_pattern << std::endl;
        std::cout << "  Natural Motion: " << (result.temporal_data.has_natural_motion ? "YES" : "NO") << std::endl;
        
        // Geometric Analysis
        std::cout << "3D Geometry:" << std::endl;
        std::cout << "  Curvature: " << std::fixed << std::setprecision(3) << result.geometric_data.face_curvature << std::endl;
        std::cout << "  Edge Alignment: " << result.geometric_data.depth_edge_alignment << std::endl;
        std::cout << "  Surface Variance: " << result.geometric_data.surface_normal_variance << std::endl;
        std::cout << "  3D Structure: " << (result.geometric_data.has_3d_structure ? "YES" : "NO") << std::endl;
        
        // Spectral Analysis
        std::cout << "Spectral Analysis:" << std::endl;
        std::cout << "  IR Ratio: " << std::fixed << std::setprecision(3) << result.spectral_data.ir_spectral_ratio << std::endl;
        std::cout << "  Display Emission: " << result.spectral_data.display_emission_detection << std::endl;
        std::cout << "  Skin Signature: " << (result.spectral_data.has_skin_spectral_signature ? "YES" : "NO") << std::endl;
        
        // Final Decision
        std::cout << "FINAL DECISION: " << (result.is_live ? "✅ LIVE HUMAN" : "❌ SPOOF ATTACK") << std::endl;
        std::cout << "  Overall Score: " << std::fixed << std::setprecision(3) << result.overall_liveness_score << std::endl;
        std::cout << "  Confidence: " << result.confidence << std::endl;
        if (!result.rejection_reason.empty()) {
            std::cout << "  Rejection Reason: " << result.rejection_reason << std::endl;
        }
    }
    
    void print_progress() {
        double face_rate = total_frames_ > 0 ? (double)face_detected_ / total_frames_ * 100.0 : 0.0;
        double live_rate = total_frames_ > 0 ? (double)live_detected_ / total_frames_ * 100.0 : 0.0;
        double spoof_rate = total_frames_ > 0 ? (double)spoof_detected_ / total_frames_ * 100.0 : 0.0;
        
        std::cout << "\n📊 Progress Update (Frame " << total_frames_ << ")" << std::endl;
        std::cout << "  Face Detection: " << std::fixed << std::setprecision(1) << face_rate << "% (" 
                  << face_detected_ << "/" << total_frames_ << ")" << std::endl;
        std::cout << "  Live Detection: " << live_rate << "% (" 
                  << live_detected_ << "/" << total_frames_ << ")" << std::endl;
        std::cout << "  Spoof Detection: " << spoof_rate << "% (" 
                  << spoof_detected_ << "/" << total_frames_ << ")" << std::endl;
    }
    
    void cleanup() {
        if (camera_running_) {
            producer_->stop();
            camera_running_ = false;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        
        std::cout << "\n🎯 FINAL RESULTS" << std::endl;
        std::cout << "================" << std::endl;
        std::cout << "Total Frames Processed: " << total_frames_ << std::endl;
        std::cout << "Processing Time: " << duration.count() << " seconds" << std::endl;
        
        double face_rate = total_frames_ > 0 ? (double)face_detected_ / total_frames_ * 100.0 : 0.0;
        double live_rate = total_frames_ > 0 ? (double)live_detected_ / total_frames_ * 100.0 : 0.0;
        double spoof_rate = total_frames_ > 0 ? (double)spoof_detected_ / total_frames_ * 100.0 : 0.0;
        
        std::cout << "\nDetection Rates:" << std::endl;
        std::cout << "  Face Detection: " << std::fixed << std::setprecision(1) << face_rate << "%" << std::endl;
        std::cout << "  Live Detection: " << live_rate << "%" << std::endl;
        std::cout << "  Spoof Detection: " << spoof_rate << "%" << std::endl;
        
        std::cout << "\n💡 Analysis:" << std::endl;
        if (live_rate > 50.0) {
            std::cout << "  ✅ System correctly identifies human faces as LIVE" << std::endl;
        } else if (live_rate < 20.0) {
            std::cout << "  ❌ System incorrectly rejects human faces (too strict)" << std::endl;
        } else {
            std::cout << "  ⚠️  System has mixed results - needs tuning" << std::endl;
        }
    }
    
};

int main() {
    DebugAntiSpoofing debug_test;
    debug_test.run();
    return 0;
}
