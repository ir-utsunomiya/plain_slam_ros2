/*
 * plain_slam_ros2
 * 
 * Copyright (c) 2025 Naoki Akai
 * All rights reserved.
 *
 * This software is provided free of charge for academic and personal use only.
 * Commercial use is strictly prohibited without prior written permission from the author.
 *
 * Conditions:
 *   - Non-commercial use only
 *   - Attribution required in any academic or derivative work
 *   - Redistribution permitted with this license header intact
 *
 * Disclaimer:
 *   This software is provided "as is" without warranty of any kind.
 *   The author is not liable for any damages arising from its use.
 *
 * For commercial licensing inquiries, please contact:
 *   Naoki Akai
 *   Email: n.akai.goo[at]gmail.com   ([at] -> @)
 *   Subject: [plain_slam_ros2] Commercial License Inquiry
 */

#include <plain_slam/lie_algebra.hpp>

namespace pslam {

Eigen::Matrix<float, 6, 6> AdjointSE3(const Sophus::SE3f& T) {
  const Eigen::Matrix3f& R = T.rotationMatrix();
  const Eigen::Vector3f& t = T.translation();

  Eigen::Matrix<float, 6, 6> Adj = Eigen::Matrix<float, 6, 6>::Zero();
  Adj.block<3,3>(0,0) = R;
  Adj.block<3,3>(3,3) = R;
  Adj.block<3,3>(0,3) = Sophus::SO3f::hat(t) * R;

  return Adj;
}

Eigen::Matrix3f LeftJacobianSO3(const Eigen::Vector3f& phi) {
  const float t = phi.norm();
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f S = Sophus::SO3f::hat(phi);

  if (t < 1e-5f) {
    return I + 0.5f * S;
  }

  const float t2 = t * t;
  const float s = sin(t);
  const float c = cos(t);

  return I + (1.0f - c) / t2 * S + (t - s) / (t2 * t) * S * S;
}

Eigen::Matrix3f LeftJacobianInvSO3(const Eigen::Vector3f& phi) {
  const float t = phi.norm();
  const Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  const Eigen::Matrix3f S = Sophus::SO3f::hat(phi);

  if (t < 1e-5f) {
    return I - 0.5f * S;
  }

  const float t2 = t * t;
  const float cot = 1.0f / tan(0.5f * t);

  return I - 0.5f * S + (1.0f - t * cot / 2.0f) / t2 * S * S;
}

} // namespace pslam
