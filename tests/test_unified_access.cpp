#include "FrameBox.hpp"
#include <iostream>
#include <cassert>

int main() {
    std::cout << "=== Testing Unified Access System ===" << std::endl;
    
    // Create a test FrameBox with sample data
    mdai::FrameBox frame;
    frame.sequence_id = 12345;
    frame.depth_width = 640;
    frame.depth_height = 480;
    frame.color_width = 640;
    frame.color_height = 480;
    frame.depth_scale = 0.001f;
    
    // Add some test data
    frame.depth_data.resize(640 * 480, 1000); // 1 meter depth
    frame.color_data.resize(640 * 480 * 3, 128); // Gray color
    frame.ir_left_data.resize(640 * 480, 64); // Dark gray IR
    
    std::cout << "✓ Created test FrameBox with sample data" << std::endl;
    
    // Test unified access methods
    std::cout << "\n--- Testing Unified Access Methods ---" << std::endl;
    
    // Test pointer access
    const uint16_t* depth_ptr = frame.get_depth_ptr();
    const uint8_t* color_ptr = frame.get_color_ptr();
    assert(depth_ptr != nullptr);
    assert(color_ptr != nullptr);
    std::cout << "✓ get_depth_ptr() and get_color_ptr() work" << std::endl;
    
    // Test vector access
    const auto& depth_vec = frame.get_depth_vector();
    const auto& color_vec = frame.get_color_vector();
    const auto& ir_left_vec = frame.get_ir_left_vector();
    assert(depth_vec.size() == 640 * 480);
    assert(color_vec.size() == 640 * 480 * 3);
    assert(ir_left_vec.size() == 640 * 480);
    std::cout << "✓ get_*_vector() methods work" << std::endl;
    
    // Test numpy-compatible access
    auto [depth_data_ptr, depth_w, depth_h, depth_dtype] = frame.get_depth_numpy();
    auto [color_data_ptr, color_w, color_h, color_dtype] = frame.get_color_numpy();
    assert(depth_data_ptr != nullptr);
    assert(color_data_ptr != nullptr);
    assert(depth_w == 640 && depth_h == 480);
    assert(color_w == 640 && color_h == 480);
    assert(std::string(depth_dtype) == "uint16");
    assert(std::string(color_dtype) == "uint8");
    std::cout << "✓ get_*_numpy() methods work" << std::endl;
    
    // Test JSON serialization
    std::string json = frame.to_json();
    assert(json.find("\"sequence_id\":12345") != std::string::npos);
    assert(json.find("\"depth_width\":640") != std::string::npos);
    assert(json.find("\"has_depth\":true") != std::string::npos);
    std::cout << "✓ to_json() works: " << json.substr(0, 100) << "..." << std::endl;
    
    // Test binary serialization
    std::vector<uint8_t> binary = frame.to_binary();
    assert(binary.size() > 0);
    std::cout << "✓ to_binary() works: " << binary.size() << " bytes" << std::endl;
    
    // Test binary deserialization
    mdai::FrameBox frame2;
    bool success = frame2.from_binary(binary);
    assert(success);
    assert(frame2.sequence_id == frame.sequence_id);
    assert(frame2.depth_width == frame.depth_width);
    assert(frame2.depth_height == frame.depth_height);
    assert(frame2.depth_data.size() == frame.depth_data.size());
    std::cout << "✓ from_binary() works" << std::endl;
    
    // Test depth access
    float depth_value = frame.get_depth_at(320, 240); // Center pixel
    assert(depth_value == 1.0f); // 1000 * 0.001 = 1.0 meters
    std::cout << "✓ get_depth_at() works: " << depth_value << " meters" << std::endl;
    
    // Test 3D point deprojection
    auto point = frame.deproject_pixel_to_point(320, 240);
    std::cout << "✓ deproject_pixel_to_point() works: [" 
              << point[0] << ", " << point[1] << ", " << point[2] << "]" << std::endl;
    
    std::cout << "\n🎉 All unified access system tests passed!" << std::endl;
    return 0;
}

