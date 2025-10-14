#pragma once

#include "FrameBox.hpp"
#include <string>

#ifdef HAVE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace mdai {
namespace utils {

#ifdef HAVE_OPENCV
/** Convert RealSense depth frame to OpenCV Mat (CV_16UC1) */
cv::Mat depth_frame_to_mat(const rs2::depth_frame& frame);
/** Convert RealSense color frame to OpenCV Mat (CV_8UC3, BGR) */
cv::Mat color_frame_to_mat(const rs2::video_frame& frame);
/** Convert RealSense IR frame to OpenCV Mat (CV_8UC1) */
cv::Mat ir_frame_to_mat(const rs2::video_frame& frame);
/** Save depth frame as PNG (16-bit grayscale) */
bool save_depth_as_png(const rs2::depth_frame& frame, const std::string& filename);
/** Save depth frame as colorized PNG (using colormap) */
bool save_depth_colorized(const rs2::depth_frame& frame,
                          const std::string& filename,
                          float min_dist = 0.0f,
                          float max_dist = 10.0f);
/** Save color frame as JPEG */
bool save_color_as_jpeg(const rs2::video_frame& frame,
                        const std::string& filename,
                        int quality = 95);
/** Save color frame as PNG */
bool save_color_as_png(const rs2::video_frame& frame, const std::string& filename);
/** Save IR frame as PNG */
bool save_ir_as_png(const rs2::video_frame& frame, const std::string& filename);
/** Save entire FrameBox (depth, color, IR) to directory */
bool save_framebox(const FrameBox& framebox,
                   const std::string& directory,
                   const std::string& prefix = "frame");
/** Export point cloud from depth frame to PLY file */
bool export_pointcloud_ply(const rs2::depth_frame& depth,
                           const rs2::video_frame& color,
                           const rs2_intrinsics& intrinsics,
                           float depth_scale,
                           const std::string& filename);
/** Create a visualization image with depth and color side-by-side */
cv::Mat create_visualization(const FrameBox& framebox);
/** Apply colormap to depth image */
cv::Mat colorize_depth(const cv::Mat& depth_mat,
                       float min_val = 0.0f,
                       float max_val = 10.0f,
                       int colormap = cv::COLORMAP_JET);
#endif

/**
 * @brief Get current timestamp string (for filenames)
 * @param format Format string (default: "%Y%m%d_%H%M%S")
 * @return Timestamp string
 */
std::string get_timestamp_string(const std::string& format = "%Y%m%d_%H%M%S");

/**
 * @brief Create directory if it doesn't exist
 * @param path Directory path
 * @return true if directory exists or was created
 */
bool ensure_directory_exists(const std::string& path);

} // namespace utils
} // namespace mdai

