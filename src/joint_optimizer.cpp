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
 *   Email: n.akai.goo[at]gmail.com   ([at] → @)
 *   Subject: [plain_slam_ros2] Commercial License Inquiry
 */

#include <plain_slam/joint_optimizer.hpp>

namespace pslam {

JointOptimizer::JointOptimizer() {
  huber_delta_ = 0.5f;
}

JointOptimizer::~JointOptimizer() {

}

bool JointOptimizer::Estimate(
  const State& pred_state,
  const StateCov& pred_state_cov,
  const PointCloud3f& scan_cloud,
  size_t max_iter_num,
  size_t num_max_matching_points,
  float max_correspondence_dist,
  float convergence_th,
  State& updated_state,
  StateCov& updated_cov,
  NormalMap& normal_map) {
  const State initial_state = pred_state;
  const Eigen::Matrix3f initial_R = initial_state.T.rotationMatrix();

  size_t scan_step = scan_cloud.size() / num_max_matching_points;
  if (scan_step < 1) {
    scan_step = 1;
  }

  State state = pred_state;
  StateCov cov = pred_state_cov;
  bool has_converged = false;

  using Matrix1x6f = Eigen::Matrix<float, 1, 6>;

  for (size_t iter_num = 0; iter_num < max_iter_num; ++iter_num) {
    size_t used_points_num = 0;
    size_t corresp_num = 0;

    std::vector<float> zs;
    std::vector<Matrix1x6f, Eigen::aligned_allocator<Matrix1x6f>> Hois;
    std::vector<Matrix1x6f, Eigen::aligned_allocator<Matrix1x6f>> Hils;
    zs.reserve(num_max_matching_points);
    Hois.reserve(num_max_matching_points);
    Hils.reserve(num_max_matching_points);

    const Sophus::SE3f Tli = state.Til.inverse();
    const Eigen::Matrix3f Roi = state.T.rotationMatrix();
    const Eigen::Matrix3f Ril = state.Til.rotationMatrix();

    for (size_t i = 0; i < scan_cloud.size(); i += scan_step) {
      used_points_num++;

      const Eigen::Vector3f query = state.T * scan_cloud[i];
      Eigen::Vector3f target, normal;
      if (!normal_map.FindCorrespondence(query, max_correspondence_dist, target, normal)) {
        continue;
      }

      const float z = (target - query).transpose() * normal;
      zs.emplace_back(z);

      const Eigen::Vector3f dz_dtoi = -normal;
      const Eigen::Vector3f Rpi = state.T.rotationMatrix() * scan_cloud[i];
      const Eigen::Matrix<float, 1, 3> nT = normal.transpose();
      const Eigen::Vector3f dz_dRoi = nT * Sophus::SO3f::hat(Rpi).matrix();
      Eigen::Matrix<float, 1, 6> Hoi;
      Hoi << dz_dtoi(0), dz_dtoi(1), dz_dtoi(2), dz_dRoi(0), dz_dRoi(1), dz_dRoi(2);
      Hois.emplace_back(Hoi);

      const Eigen::Vector3f dz_dtil = nT * Roi;
      const Eigen::Vector3f pl = Tli * scan_cloud[i];
      const Eigen::Vector3f dz_dRil = nT * Roi * Sophus::SO3f::hat(Ril * pl).matrix();
      Eigen::Matrix<float, 1, 6> Hil;
      Hil << dz_dtil(0), dz_dtil(1), dz_dtil(2), dz_dRil(0), dz_dRil(1), dz_dRil(2);
      Hils.emplace_back(Hil);

      corresp_num++;
      if (corresp_num >= num_max_matching_points) {
        break;
      }
    }

    active_points_rate_ = static_cast<float>(corresp_num) / static_cast<float>(used_points_num);

    const float rinv = 1e-6f;
    Eigen::Matrix<float, Eigen::Dynamic, 1> z(corresp_num, 1);
    Eigen::Matrix<float, Eigen::Dynamic, 24> H = Eigen::Matrix<float, Eigen::Dynamic, 24>::Zero(corresp_num, 24);
    Eigen::Matrix<float, 24, Eigen::Dynamic> HT = Eigen::Matrix<float, 24, Eigen::Dynamic>::Zero(24, corresp_num);
    // Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Rinv = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(corresp_num, corresp_num);;
    for (size_t i = 0; i < corresp_num; ++i) {
      if (huber_delta_ > 0.0f) {
        const float abs_z = std::abs(zs[i]);
        const float w = (abs_z <= huber_delta_) ? 1.0f : (huber_delta_ / abs_z);
        z(i, 0) = w * zs[i];
        H.block<1, 6>(i, 0) = w * Hois[i];
        HT.block<6, 1>(0, i) = w * Hois[i].transpose();
        H.block<1, 6>(i, 18) = w * Hils[i];
        HT.block<6, 1>(18, i) = w * Hils[i].transpose();
      } else {
        z(i, 0) = zs[i];
        H.block<1, 6>(i, 0) = Hois[i];
        HT.block<6, 1>(0, i) = Hois[i].transpose();
        H.block<1, 6>(i, 18) = Hils[i];
        HT.block<6, 1>(18, i) = Hils[i].transpose();
      }
      // Rinv(i, i) = rinv;
    }

    Eigen::Matrix<float, 24, 24> I = Eigen::Matrix<float, 24, 24>::Identity();

    Eigen::Matrix<float, 24, 24> J = I;
    const Eigen::Matrix3f dR = initial_R.transpose() * state.T.rotationMatrix();  
    const Eigen::Vector3f dtheta = Sophus::SO3f(dR).log();
    J.block<3, 3>(3, 3) = LeftJacobianInvSO3(dtheta).transpose();
    Eigen::Matrix<float, 24, 24> Jinv = J.ldlt().solve(I);

    Eigen::Matrix<float, 24, 24> P = Jinv * cov * Jinv.transpose();
    Eigen::Matrix<float, 24, 24> Pinv = P.ldlt().solve(I);

    // Eigen::Matrix<float, 24, Eigen::Dynamic> HT = H.transpose();
    // Eigen::Matrix<float, 24, 24> HTRinvHPinv = HT * Rinv * H + Pinv;
    Eigen::Matrix<float, 24, 24> HTRinvHPinv = rinv * HT * H + Pinv;
    Eigen::Matrix<float, 24, 24> denom = HTRinvHPinv.ldlt().solve(I);
    // Eigen::Matrix<float, 24, Eigen::Dynamic> K = denom * HT * Rinv;
    Eigen::Matrix<float, 24, Eigen::Dynamic> K = rinv * denom * HT;

    Eigen::Matrix<float, 24, 1> dx;
    dx.block<3, 1>(0, 0) = state.T.translation() - initial_state.T.translation();
    dx.block<3, 1>(3, 0) = (initial_state.T.so3().inverse() * state.T.so3()).log();
    dx.block<3, 1>(6, 0) = state.v - initial_state.v;
    dx.block<3, 1>(9, 0) = state.gb - initial_state.gb;
    dx.block<3, 1>(12, 0) = state.ab - initial_state.ab;
    dx.block<3, 1>(15, 0) = state.gacc - initial_state.gacc;
    dx.block<3, 1>(18, 0) = state.Til.translation() - initial_state.Til.translation();
    dx.block<3, 1>(21, 0) = (initial_state.Til.so3().inverse() * state.Til.so3()).log();

    Eigen::Matrix<float, 24, 1> update = -K * z - (I - K * H) * Jinv * dx;
    const Eigen::Vector3f t_new = state.T.translation() + update.block<3, 1>(0, 0);
    const Sophus::SO3f R_new = Sophus::SO3f::exp(update.block<3, 1>(3, 0)) * state.T.so3();
    state.T = Sophus::SE3f(R_new, t_new);
    state.v += update.block<3, 1>(6, 0);
    state.gb += update.block<3, 1>(9, 0);
    state.ab += update.block<3, 1>(12, 0);
    state.gacc += update.block<3, 1>(15, 0);
    const Eigen::Vector3f til_new = state.Til.translation() + update.block<3, 1>(18, 0);
    const Sophus::SO3f Ril_new = Sophus::SO3f::exp(update.block<3, 1>(21, 0)) * state.Til.so3();
    state.Til = Sophus::SE3f(Ril_new, til_new);

    cov = (I - K * H) * P;

    const float epsilon = update.norm();
    std::cout << "iteration = " << iter_num + 1 << ", epsilon = " << epsilon << std::endl;
    if (epsilon < convergence_th) {
      has_converged = true;
      break;
    }
  }

  if (!has_converged) {
    return false;
  }

  updated_state = state;
  updated_cov = cov;

  return true;
}

} // namespace pslam
