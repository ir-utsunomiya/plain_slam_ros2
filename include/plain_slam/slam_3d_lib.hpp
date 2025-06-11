#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <filesystem>

#include <Eigen/Core>

#include <sophus/se3.hpp>

#include <plain_slam/types.hpp>
#include <plain_slam/io.hpp>
#include <plain_slam/keyframe_detector.hpp>
#include <plain_slam/voxel_grid_filter.hpp>
#include <plain_slam/gicp.hpp>

namespace pslam {

class SLAM3DLib {
 public:
  SLAM3DLib();

  ~SLAM3DLib();

  void SetData(
    const Sophus::SE3f& odom_pose,
    const PointCloud3f& scan_cloud,
    const std::vector<float>& scan_intensities);

  void SetSLAMDataDir(const std::string& dir) {
    slam_data_dir_ = dir;
  }

  void SetAccumulationCycle(size_t cycle) {
    accumulation_cycle_ = cycle;
  }

  void SetScanCloudFilterSize(float size) {
    scan_cloud_filter_size_ = size;
  }

  void SetMaxNumLoopCandidates(size_t num) {
    max_num_loop_candidates_ = num;
  }

  void SetLoopDetectionDistanceThresh(float th) {
    loop_detection_distance_thresh_ = th;
  }

  bool IsPoseGraphUpdated() const {
    return is_pose_graph_updated_;
  }

  void SetPoseGraphUpdated(bool flag) {
    is_pose_graph_updated_ = flag;
  }

  Sophus::SE3f GetSLAMPose() const {
    return slam_pose_;
  }

  void GetGraphNodes(PointCloud3f& nodes);

  void GetOdomEdgePoints(
    PointCloud3f& start_points,
    PointCloud3f& end_points);

  void GetLoopEdgePoints(
    PointCloud3f& start_points,
    PointCloud3f& end_points);

  const PointCloud3f& GetMapCloud() const {
    return map_cloud_;
  }

 private:
  std::string slam_data_dir_;

  Sophus::SE3f slam_pose_;
  Sophus::SE3f prev_odom_pose_;
  bool is_odom_pose_initialized_;

  PointCloud3f map_cloud_;

  size_t accumulation_cycle_;
  size_t accumulated_count_;
  PointCloud3f accumulated_scan_cloud_;
  std::vector<float> accumulated_scan_intensities_;

  float scan_cloud_filter_size_;

  KeyframeDetector kf_detector_;

  std::vector<Sophus::SE3f> pose_graph_;
  std::vector<Edge> odom_edges_;
  std::vector<Edge> loop_edges_;

  PointCloud3f graph_pose_points_;
  std::unique_ptr<PointCloudAdaptor> graph_pose_adaptor_;
  std::unique_ptr<KDTree> graph_pose_kdtree_;

  size_t max_num_loop_candidates_;
  float loop_detection_distance_thresh_;

  bool is_pose_graph_updated_;

  void GetCloudFileNames(
    size_t idx,
    std::string& raw_cloud_file,
    std::string& filtered_cloud_file);

  void BuildMapCloud();

  void BuildGraphPoseKDTree(const Sophus::SE3f& pose);

  void GetTargetCandidateIndices(
    const Sophus::SE3f& source_pose,
    std::vector<size_t>& target_indices);

  void TransformCloud(
    const Sophus::SE3f& T,
    PointCloud3f& cloud);

  void DumpCloud(
    const std::string& fname,
    const PointCloud3f& cloud) {
    FILE* fp = fopen(fname.c_str(), "w");
    for (const auto& p: cloud) {
      fprintf(fp, "%f %f %f\n", p.x(), p.y(), p.z());
    }
    fclose(fp);
  }
};


} // namespace pslam