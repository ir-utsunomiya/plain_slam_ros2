#pragma once

#include <vector>
#include <cmath>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include <boost/circular_buffer.hpp>

#include <plain_slam/types.hpp>

namespace pslam {

using Jacobians = std::vector<Eigen::Matrix<float, 1, 6>,
                              Eigen::aligned_allocator<Eigen::Matrix<float, 1, 6>>>;

class ScanIntensityMatcher {
 public:
  ScanIntensityMatcher();

  ~ScanIntensityMatcher();

  void AddFrame(
    const Sophus::SE3f& pose,
    const PointCloud3f& aligned_scan_cloud,
    const std::vector<float>& scan_intensities) {
    poses_.push_back(pose);
    aligned_scan_clouds_.push_back(aligned_scan_cloud);
    scan_intensities_.push_back(scan_intensities);
  }

  void MakeScanImage(const State& state);

  void ComputeResidualsAndJacobians(
    const State& state,
    const PointCloud3f& scan_cloud,
    const std::vector<float>& scan_intensities,
    std::vector<float>& zs,
    Jacobians& Hois,
    Jacobians& Hils);

 private:
  boost::circular_buffer<Sophus::SE3f> poses_;
  boost::circular_buffer<PointCloud3f> aligned_scan_clouds_;
  boost::circular_buffer<std::vector<float>> scan_intensities_;

  size_t image_width_;
  size_t image_height_;
  float vertical_fov_;

  float max_scan_intensity_;
  Sophus::SE3f origin_pose_;
  std::vector<std::vector<float>> scan_image_;
  std::vector<std::vector<float>> depth_map_;

  void point2pixel(
    const Eigen::Vector3f& point,
    Eigen::Vector2i& pixel);

  bool IsDepthEffective(
    int u,
    int v);
};

} // namespace pslam