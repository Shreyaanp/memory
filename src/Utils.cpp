#include "Utils.hpp"
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace mdai {
namespace utils {

cv::Mat depth_frame_to_mat(const rs2::depth_frame& frame) {
    if (!frame) return cv::Mat();
    
    int width = frame.get_width();
    int height = frame.get_height();
    
    // Depth is 16-bit unsigned
    cv::Mat mat(height, width, CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    return mat.clone();
}

cv::Mat color_frame_to_mat(const rs2::video_frame& frame) {
    if (!frame) return cv::Mat();
    
    int width = frame.get_width();
    int height = frame.get_height();
    
    // Color is BGR8
    cv::Mat mat(height, width, CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    return mat.clone();
}

cv::Mat ir_frame_to_mat(const rs2::video_frame& frame) {
    if (!frame) return cv::Mat();
    
    int width = frame.get_width();
    int height = frame.get_height();
    
    // IR is 8-bit grayscale
    cv::Mat mat(height, width, CV_8UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    return mat.clone();
}

bool save_depth_as_png(const rs2::depth_frame& frame, const std::string& filename) {
    if (!frame) return false;
    
    cv::Mat mat = depth_frame_to_mat(frame);
    if (mat.empty()) return false;
    
    return cv::imwrite(filename, mat);
}

bool save_depth_colorized(const rs2::depth_frame& frame, 
                          const std::string& filename,
                          float min_dist,
                          float max_dist) {
    if (!frame) return false;
    
    cv::Mat depth_mat = depth_frame_to_mat(frame);
    if (depth_mat.empty()) return false;
    
    cv::Mat colorized = colorize_depth(depth_mat, min_dist, max_dist);
    return cv::imwrite(filename, colorized);
}

bool save_color_as_jpeg(const rs2::video_frame& frame, 
                        const std::string& filename,
                        int quality) {
    if (!frame) return false;
    
    cv::Mat mat = color_frame_to_mat(frame);
    if (mat.empty()) return false;
    
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(quality);
    
    return cv::imwrite(filename, mat, params);
}

bool save_color_as_png(const rs2::video_frame& frame, const std::string& filename) {
    if (!frame) return false;
    
    cv::Mat mat = color_frame_to_mat(frame);
    if (mat.empty()) return false;
    
    return cv::imwrite(filename, mat);
}

bool save_ir_as_png(const rs2::video_frame& frame, const std::string& filename) {
    if (!frame) return false;
    
    cv::Mat mat = ir_frame_to_mat(frame);
    if (mat.empty()) return false;
    
    return cv::imwrite(filename, mat);
}

bool save_framebox(const FrameBox& framebox, 
                   const std::string& directory,
                   const std::string& prefix) {
    if (!ensure_directory_exists(directory)) {
        return false;
    }
    
    std::string base = directory + "/" + prefix + "_" + 
                       std::to_string(framebox.sequence_id);
    
    bool success = true;
    
    // Save depth using unified access system
    if (framebox.get_depth_ptr()) {
        cv::Mat depth_mat = framebox.get_depth_mat();
        if (!depth_mat.empty()) {
            success &= cv::imwrite(base + "_depth.png", depth_mat);
            // Create colorized depth for visualization
            cv::Mat depth_colorized;
            depth_mat.convertTo(depth_colorized, CV_8UC1, 255.0 / 65535.0);
            cv::applyColorMap(depth_colorized, depth_colorized, cv::COLORMAP_JET);
            success &= cv::imwrite(base + "_depth_color.png", depth_colorized);
        }
    }
    
    // Save color using unified access system
    if (framebox.get_color_ptr()) {
        cv::Mat color_mat = framebox.get_color_mat();
        if (!color_mat.empty()) {
            success &= cv::imwrite(base + "_color.jpg", color_mat);
        }
    }
    
    // Save IR using unified access system
    if (framebox.get_ir_left_vector().size() > 0) {
        const auto& ir_data = framebox.get_ir_left_vector();
        if (framebox.ir_width > 0 && framebox.ir_height > 0) {
            cv::Mat ir_mat(framebox.ir_height, framebox.ir_width, CV_8UC1, 
                          const_cast<uint8_t*>(ir_data.data()));
            success &= cv::imwrite(base + "_ir_left.png", ir_mat);
        }
    }
    if (framebox.get_ir_right_vector().size() > 0) {
        const auto& ir_data = framebox.get_ir_right_vector();
        if (framebox.ir_width > 0 && framebox.ir_height > 0) {
            cv::Mat ir_mat(framebox.ir_height, framebox.ir_width, CV_8UC1, 
                          const_cast<uint8_t*>(ir_data.data()));
            success &= cv::imwrite(base + "_ir_right.png", ir_mat);
        }
    }
    
    return success;
}

bool export_pointcloud_ply(const rs2::depth_frame& depth,
                           const rs2::video_frame& color,
                           const rs2_intrinsics& intrinsics,
                           float depth_scale,
                           const std::string& filename) {
    if (!depth) return false;
    
    int width = depth.get_width();
    int height = depth.get_height();
    
    // Get data pointers
    const uint16_t* depth_data = (const uint16_t*)depth.get_data();
    const uint8_t* color_data = color ? (const uint8_t*)color.get_data() : nullptr;
    
    // Count valid points
    int valid_points = 0;
    for (int i = 0; i < width * height; ++i) {
        if (depth_data[i] > 0) valid_points++;
    }
    
    // Open file
    std::ofstream ofs(filename);
    if (!ofs) return false;
    
    // Write PLY header
    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "element vertex " << valid_points << "\n";
    ofs << "property float x\n";
    ofs << "property float y\n";
    ofs << "property float z\n";
    if (color_data) {
        ofs << "property uchar red\n";
        ofs << "property uchar green\n";
        ofs << "property uchar blue\n";
    }
    ofs << "end_header\n";
    
    // Write points
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            uint16_t depth_value = depth_data[idx];
            
            if (depth_value == 0) continue;
            
            // Deproject to 3D
            float depth_meters = depth_value * depth_scale;
            float pixel[2] = {static_cast<float>(x), static_cast<float>(y)};
            float point[3];
            rs2_deproject_pixel_to_point(point, &intrinsics, pixel, depth_meters);
            
            ofs << point[0] << " " << point[1] << " " << point[2];
            
            if (color_data) {
                int color_idx = idx * 3;
                // BGR to RGB
                ofs << " " << (int)color_data[color_idx + 2] 
                    << " " << (int)color_data[color_idx + 1] 
                    << " " << (int)color_data[color_idx + 0];
            }
            
            ofs << "\n";
        }
    }
    
    ofs.close();
    return true;
}

cv::Mat create_visualization(const FrameBox& framebox) {
    cv::Mat depth_vis, color_vis;
    
    // Get depth visualization using unified access system
    if (framebox.get_depth_ptr()) {
        cv::Mat depth_mat = framebox.get_depth_mat();
        if (!depth_mat.empty()) {
            depth_vis = colorize_depth(depth_mat);
        }
    }
    
    // Get color visualization using unified access system
    if (framebox.get_color_ptr()) {
        color_vis = framebox.get_color_mat();
    }
    
    // Combine side by side
    if (!depth_vis.empty() && !color_vis.empty()) {
        // Resize to same height if needed
        if (depth_vis.rows != color_vis.rows) {
            cv::resize(depth_vis, depth_vis, color_vis.size());
        }
        
        cv::Mat combined;
        cv::hconcat(depth_vis, color_vis, combined);
        
        // Add text info
        std::string info = "Seq: " + std::to_string(framebox.sequence_id);
        cv::putText(combined, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        
        return combined;
    } else if (!depth_vis.empty()) {
        return depth_vis;
    } else if (!color_vis.empty()) {
        return color_vis;
    }
    
    return cv::Mat();
}

cv::Mat colorize_depth(const cv::Mat& depth_mat,
                      float min_val,
                      float max_val,
                      int colormap) {
    if (depth_mat.empty()) return cv::Mat();
    
    // Normalize to 0-255
    cv::Mat normalized;
    depth_mat.convertTo(normalized, CV_8UC1, 255.0 / (max_val - min_val), -min_val * 255.0 / (max_val - min_val));
    
    // Apply colormap
    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, colormap);
    
    return colorized;
}

std::string get_timestamp_string(const std::string& format) {
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, format.c_str());
    return oss.str();
}

bool ensure_directory_exists(const std::string& path) {
    struct stat info;
    
    if (stat(path.c_str(), &info) == 0) {
        return S_ISDIR(info.st_mode);
    }
    
    // Try to create directory
    return mkdir(path.c_str(), 0755) == 0;
}

} // namespace utils
} // namespace mdai

