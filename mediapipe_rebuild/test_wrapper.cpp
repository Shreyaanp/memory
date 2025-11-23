/**
 * Test program for MediaPipe Face Mesh Wrapper
 * Captures from RealSense and runs face detection
 */

#include "face_mesh_wrapper.h"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <chrono>

using namespace mdai;

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "MediaPipe Face Mesh Wrapper - Test" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    // Initialize detector
    FaceMeshConfig config;
    config.num_faces = 1;
    config.min_detection_confidence = 0.5f;
    config.min_tracking_confidence = 0.5f;
    config.use_gpu = true; // MediaPipe Python uses GPU automatically if available
    
    std::cout << "[1/3] Initializing MediaPipe Face Mesh detector..." << std::endl;
    FaceMeshDetector detector(config);
    
    if (!detector.IsInitialized()) {
        std::cerr << "ERROR: Failed to initialize detector: " 
                  << detector.GetLastError() << std::endl;
        return 1;
    }
    std::cout << "✓ Detector initialized" << std::endl;
    std::cout << std::endl;
    
    // Initialize RealSense
    std::cout << "[2/3] Initializing RealSense camera..." << std::endl;
    rs2::pipeline pipe;
    rs2::config rs_config;
    
    // Low res for testing
    rs_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    
    try {
        pipe.start(rs_config);
        std::cout << "✓ Camera initialized (640x480 @ 30fps)" << std::endl;
    } catch (const rs2::error& e) {
        std::cerr << "ERROR: Failed to start camera: " << e.what() << std::endl;
        return 1;
    }
    
    // Wait for auto-exposure
    std::cout << "  Warming up camera..." << std::endl;
    for (int i = 0; i < 30; i++) {
        pipe.wait_for_frames();
    }
    std::cout << "✓ Camera ready" << std::endl;
    std::cout << std::endl;
    
    // Process loop
    std::cout << "[3/3] Processing frames..." << std::endl;
    std::cout << "Press 'q' to quit" << std::endl;
    std::cout << std::endl;
    
    int frame_count = 0;
    int face_detected_count = 0;
    double total_inference_time = 0.0;
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        // Capture frame
        auto frames = pipe.wait_for_frames();
        auto color_frame = frames.get_color_frame();
        
        if (!color_frame) continue;
        
        // Convert to cv::Mat
        cv::Mat frame(
            cv::Size(color_frame.get_width(), color_frame.get_height()),
            CV_8UC3,
            (void*)color_frame.get_data(),
            cv::Mat::AUTO_STEP
        );
        
        // Run detection
        FaceMeshResult result;
        auto t1 = std::chrono::high_resolution_clock::now();
        bool success = detector.Detect(frame, result);
        auto t2 = std::chrono::high_resolution_clock::now();
        
        double inference_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        total_inference_time += inference_ms;
        
        if (!success) {
            std::cerr << "Detection failed: " << detector.GetLastError() << std::endl;
            break;
        }
        
        // Draw results
        if (!result.landmarks.empty()) {
            face_detected_count++;
            
            // Draw bounding box
            cv::rectangle(frame, result.bbox, cv::Scalar(0, 255, 0), 2);
            
            // Draw subset of landmarks (every 10th for visibility)
            for (size_t i = 0; i < result.landmarks.size(); i += 10) {
                const auto& lm = result.landmarks[i];
                cv::circle(frame, cv::Point(lm.x, lm.y), 2, cv::Scalar(0, 0, 255), -1);
            }
            
            // Draw nose tip (landmark 4)
            if (result.landmarks.size() > 4) {
                const auto& nose = result.landmarks[4];
                cv::circle(frame, cv::Point(nose.x, nose.y), 8, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, cv::Point(nose.x - 20, nose.y), 
                        cv::Point(nose.x + 20, nose.y), cv::Scalar(0, 255, 0), 2);
                cv::line(frame, cv::Point(nose.x, nose.y - 20), 
                        cv::Point(nose.x, nose.y + 20), cv::Scalar(0, 255, 0), 2);
                
                cv::putText(frame, 
                           cv::format("Nose: (%.0f, %.0f)", nose.x, nose.y),
                           cv::Point(nose.x + 25, nose.y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
            
            // Info text
            cv::putText(frame, 
                       cv::format("Landmarks: %zu", result.landmarks.size()),
                       cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            
            cv::putText(frame, 
                       cv::format("Confidence: %.2f", result.confidence),
                       cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        } else {
            cv::putText(frame, "No face detected",
                       cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }
        
        // FPS and inference time
        cv::putText(frame, 
                   cv::format("Inference: %.1f ms", inference_ms),
                   cv::Point(10, frame.rows - 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        frame_count++;
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        double fps = frame_count / elapsed;
        
        cv::putText(frame, 
                   cv::format("FPS: %.1f", fps),
                   cv::Point(10, frame.rows - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        
        // Display
        cv::imshow("MediaPipe Face Mesh", frame);
        
        // Print stats every 30 frames
        if (frame_count % 30 == 0) {
            double avg_inference = total_inference_time / frame_count;
            double detection_rate = (double)face_detected_count / frame_count * 100.0;
            
            std::cout << "Frame " << frame_count << ": "
                     << "FPS=" << std::fixed << std::setprecision(1) << fps
                     << " | Inference=" << avg_inference << "ms"
                     << " | Detection Rate=" << detection_rate << "%"
                     << std::endl;
        }
        
        // Check for quit
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    
    // Cleanup
    pipe.stop();
    cv::destroyAllWindows();
    
    // Final stats
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Test Complete - Statistics" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total frames: " << frame_count << std::endl;
    std::cout << "Faces detected: " << face_detected_count << std::endl;
    std::cout << "Detection rate: " << std::fixed << std::setprecision(1)
              << (double)face_detected_count / frame_count * 100.0 << "%" << std::endl;
    std::cout << "Avg inference: " << total_inference_time / frame_count << " ms" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}


