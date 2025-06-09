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

#include <plain_slam/graph_optimizer.hpp>

namespace pslam {

GraphOptimizer::GraphOptimizer() {
  max_iter_num_ = 10;
  epsilon_ = 0.1f;
  huber_delta_ = 0.001f;
}

GraphOptimizer::~GraphOptimizer() {
  
}

void GraphOptimizer::Optimize(
  const std::vector<Edge>& odom_edges,
  const std::vector<Edge>& loop_edges,
  std::vector<Sophus::SE3f>& pose_graph) {
  has_converged_ = false;
  const size_t num_variables = 6 * pose_graph.size();

  const Sophus::SE3f origin_pose = pose_graph[0];

  const size_t num_triplet_list = 36 * (1 + 4 * odom_edges.size() + 4 * loop_edges.size());

  for (size_t iter_num = 0; iter_num < max_iter_num_; iter_num++) {
    std::vector<Eigen::Triplet<float>> H_triplet_list;
    H_triplet_list.reserve(num_triplet_list);
    Eigen::MatrixXf B = Eigen::MatrixXf::Zero(num_variables, 1);

    // prior for the 0th node
    // T_error = T_i^{-1} T_prior
    {
      const Eigen::Matrix<float, 6, 1> e = (origin_pose.inverse() * pose_graph[0]).log();
      const Eigen::Matrix<float, 6, 6> J = AdjointSE3(pose_graph[0].inverse());
      const Eigen::Matrix<float, 6, 6> JT = J.transpose();

      const Eigen::Matrix<float, 6, 6> W = 1000.0f * Eigen::Matrix<float, 6, 6>::Identity();
      Eigen::Matrix<float, 6, 6> H;
      Eigen::Matrix<float, 6, 1> b;
      if (huber_delta_ > 0.0f) {
        const float abs_e = e.norm();
        const float w = (abs_e <= huber_delta_) ? 1.0f : (huber_delta_ / abs_e);
        H = w * JT * W * J;
        b = w * JT * W * e;
      } else {
        H = JT * W * J;
        b = JT * W * e;
      }

      for (size_t j = 0; j < 6; ++j) {
        for (size_t k = 0; k < 6; ++k)
          H_triplet_list.emplace_back(Eigen::Triplet<float>(j, k, H(j, k)));
      }
      B.block<6, 1>(6, 0) += b;
    }

    // odom edges
    for (size_t i = 0; i < odom_edges.size(); ++i) {
      const size_t source_idx = odom_edges[i].source_idx;
      const size_t target_idx = odom_edges[i].target_idx;
      const Sophus::SE3f Eij = odom_edges[i].relative;
      const Sophus::SE3f& Ti = pose_graph[source_idx];
      const Sophus::SE3f& Tj = pose_graph[target_idx];

      const Eigen::Matrix<float, 6, 1> e = (Eij.inverse() * Ti.inverse() * Tj).log();
      const Eigen::Matrix<float, 6, 6> Ji = -1.0f * AdjointSE3(Tj.inverse());
      const Eigen::Matrix<float, 6, 6> Jj = -1.0f * Ji;
      const Eigen::Matrix<float, 6, 6> JiT = Ji.transpose();
      const Eigen::Matrix<float, 6, 6> JjT = Jj.transpose();

      Eigen::Matrix<float, 6, 6> Hii, Hij, Hjj;
      Eigen::Matrix<float, 6, 1> bi, bj;
      if (huber_delta_ > 0.0f) {
        const float abs_e = e.norm();
        const float w = (abs_e <= huber_delta_) ? 1.0f : (huber_delta_ / abs_e);
        Hii = w * JiT * Ji;
        Hij = w * JiT * Jj;
        Hjj = w * JjT * Jj;
        bi = w * JiT * e;
        bj = w * JjT * e;
      } else {
        Hii = JiT * Ji;
        Hij = JiT * Jj;
        Hjj = JjT * Jj;
        bi = JiT * e;
        bj = JjT * e;
      }

      for (size_t j = 0; j < 6; ++j) {
        for (size_t k = 0; k < 6; ++k) {
          H_triplet_list.emplace_back(Eigen::Triplet<float>(source_idx * 6 + j, source_idx * 6 + k, Hii(j, k)));
          H_triplet_list.emplace_back(Eigen::Triplet<float>(source_idx * 6 + j, target_idx * 6 + k, Hij(j, k)));
          H_triplet_list.emplace_back(Eigen::Triplet<float>(target_idx * 6 + j, source_idx * 6 + k, Hij(k, j)));
          H_triplet_list.emplace_back(Eigen::Triplet<float>(target_idx * 6 + j, target_idx * 6 + k, Hjj(j, k)));
        }
      }
      B.block<6, 1>(source_idx * 6, 0) += bi;
      B.block<6, 1>(target_idx * 6, 0) += bj;
    }

    // loop edges
    for (size_t i = 0; i < loop_edges.size(); ++i) {
      const size_t source_idx = loop_edges[i].source_idx;
      const size_t target_idx = loop_edges[i].target_idx;
      const Sophus::SE3f Eij = loop_edges[i].relative;
      const Sophus::SE3f& Ti = pose_graph[source_idx];
      const Sophus::SE3f& Tj = pose_graph[target_idx];

      const Eigen::Matrix<float, 6, 1> e = (Eij.inverse() * Ti.inverse() * Tj).log();
      const Eigen::Matrix<float, 6, 6> Ji = -1.0f * AdjointSE3(Tj.inverse());
      const Eigen::Matrix<float, 6, 6> Jj = -1.0f * Ji;
      const Eigen::Matrix<float, 6, 6> JiT = Ji.transpose();
      const Eigen::Matrix<float, 6, 6> JjT = Jj.transpose();

      Eigen::Matrix<float, 6, 6> Hii, Hij, Hjj;
      Eigen::Matrix<float, 6, 1> bi, bj;
      if (huber_delta_ > 0.0f) {
        const float abs_e = e.norm();
        const float w = (abs_e <= huber_delta_) ? 1.0f : (huber_delta_ / abs_e);
        Hii = w * JiT * Ji;
        Hij = w * JiT * Jj;
        Hjj = w * JjT * Jj;
        bi = w * JiT * e;
        bj = w * JjT * e;
      } else {
        Hii = JiT * Ji;
        Hij = JiT * Jj;
        Hjj = JjT * Jj;
        bi = JiT * e;
        bj = JjT * e;
      }

      for (size_t j = 0; j < 6; ++j) {
        for (size_t k = 0; k < 6; ++k) {
          H_triplet_list.emplace_back(Eigen::Triplet<float>(source_idx * 6 + j, source_idx * 6 + k, Hii(j, k)));
          H_triplet_list.emplace_back(Eigen::Triplet<float>(source_idx * 6 + j, target_idx * 6 + k, Hij(j, k)));
          H_triplet_list.emplace_back(Eigen::Triplet<float>(target_idx * 6 + j, source_idx * 6 + k, Hij(k, j)));
          H_triplet_list.emplace_back(Eigen::Triplet<float>(target_idx * 6 + j, target_idx * 6 + k, Hjj(j, k)));
        }
      }
      B.block<6, 1>(source_idx * 6, 0) += bi;
      B.block<6, 1>(target_idx * 6, 0) += bj;
    }

    Eigen::SparseMatrix<float> H(num_variables, num_variables);
    H.setFromTriplets(H_triplet_list.begin(), H_triplet_list.end());
    for (size_t i = 0; i < num_variables; ++i) {
      H.coeffRef(i, i) += 1e-5f;
    }

    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver(H);
    const Eigen::MatrixXf updates = solver.solve(-B);

    for (size_t i = 0; i < pose_graph.size(); ++i) {
      const Eigen::Matrix<float, 6, 1> d = updates.block<6, 1>(i * 6, 0);
      pose_graph[i] = Sophus::SE3f::exp(d) * pose_graph[i];
    }

    const float epsilon = updates.norm();
    if (epsilon < epsilon_) {
      has_converged_ = true;
      break;
    }
  }
}


} // namespace pslam
