#include <plain_slam/scan_intensity_matcher.hpp>

namespace pslam {

ScanIntensityMatcher::ScanIntensityMatcher() {
  const size_t num_frame = 10;
  poses_ = boost::circular_buffer<Sophus::SE3f>(num_frame);
  aligned_scan_clouds_ = boost::circular_buffer<PointCloud3f>(num_frame);
  scan_intensities_ = boost::circular_buffer<std::vector<float>>(num_frame);

  image_width_ = 270;
  image_height_ = 90;
  vertical_fov_ = 120.0f * M_PI / 180.0f;

  scan_image_.resize(image_width_);
  depth_map_.resize(image_width_);
  for (size_t i = 0; i < image_width_; ++i) {
    scan_image_[i].resize(image_height_);
    depth_map_[i].resize(image_height_);
  }
}

ScanIntensityMatcher::~ScanIntensityMatcher() {
  
}

void ScanIntensityMatcher::point2pixel(
  const Eigen::Vector3f& point,
  Eigen::Vector2i& pixel) {
  const float w_half = static_cast<float>(image_width_) * 0.5f;
  const float h_half = static_cast<float>(image_height_) * 0.5f;
  const float L = sqrtf(point.x() * point.x() + point.y() * point.y()) - 0.0f;
  const float R = sqrtf(L * L + point.z() * point.z());
  pixel.x() = static_cast<int>(-1.0f * w_half / M_PI * atan2(point.y(), point.x()) + w_half);
  pixel.y() = static_cast<int>(-1.0f * image_height_ / vertical_fov_ * asin(point.z() / R) + h_half);
}

void ScanIntensityMatcher::MakeScanImage(const State& state) {
  // clear
  for (size_t u = 0; u < image_width_; ++u) {
    for (size_t v = 0; v < image_height_; ++v) {
      scan_image_[u][v] = -1.0f;
      depth_map_[u][v] = -1.0f;
    }
  }

  if (poses_.size() == 0) {
    return;
  }

  // calculate normalization criteria (maximum scan intensity)
  const size_t origin_idx = poses_.size() - 1;
  float scan_intensity_sum = 0.0f;
  for (size_t i = 0; i < scan_intensities_[origin_idx].size(); ++i) {
    scan_intensity_sum += scan_intensities_[origin_idx][i];
  }
  const float scan_intensity_ave = scan_intensity_sum / scan_intensities_[origin_idx].size();

  scan_intensity_sum = 0.0f;
  for (size_t i = 0; i < scan_intensities_[origin_idx].size(); ++i) {
    const float diff = scan_intensity_ave - scan_intensities_[origin_idx][i];
    scan_intensity_sum += diff * diff;
  }
  const float scan_intensity_var = scan_intensity_sum / scan_intensities_[origin_idx].size();
  max_scan_intensity_ = scan_intensity_ave + 1.0f * sqrtf(scan_intensity_var);

  origin_pose_ = poses_[origin_idx].inverse(); // Toi^{-1} = Tio
  const Sophus::SE3f Tlo = state.Til.inverse() * origin_pose_;

  for (size_t i = 0; i < aligned_scan_clouds_.size(); ++i) {
    for (size_t j = 0; j < aligned_scan_clouds_[i].size(); ++j) {
      const Eigen::Vector3f pl = Tlo * aligned_scan_clouds_[i][j];
      Eigen::Vector2i pix;
      point2pixel(pl, pix);
      const int u = pix.x();
      const int v = pix.y();
      if (u < 0 || static_cast<int>(image_width_) <= u ||
          v < 0 || static_cast<int>(image_height_) <= v) {
        continue;
      }

      const float d = pl.norm();
      if (depth_map_[u][v] < 0.0f || d < depth_map_[u][v]) {
        float val = scan_intensities_[i][j] / max_scan_intensity_;
        if (val > 1.0f) {
          val = 1.0f;
        }
        scan_image_[u][v] = val;
        depth_map_[u][v] = d;
      }
    }
  }

  if (true) {
    std::cout << "aligned_scan_clouds_.size() = " << aligned_scan_clouds_.size() << std::endl;
    FILE* fp = fopen("/tmp/scan_image.txt", "w");
    for (size_t v = 0; v < image_height_; ++v) {
      for (size_t u = 0; u < image_width_; ++u) {
        fprintf(fp, "%lu %lu %f %f\n", u, v, scan_image_[u][v], depth_map_[u][v]);
      }
      fprintf(fp, "\n");
    }
    fclose(fp);
  }
}

bool ScanIntensityMatcher::IsDepthEffective(
  int u,
  int v) {
  if (depth_map_[u][v] > 0.0f && 
      depth_map_[u - 1][v] > 0.0f && depth_map_[u + 1][v] > 0.0f &&
      depth_map_[u][v - 1] > 0.0f && depth_map_[u][v + 1] > 0.0f)
  {
    return true;
  } else {
    return false;
  }
}

void ScanIntensityMatcher::ComputeResidualsAndJacobians(
  const State& state,
  const PointCloud3f& scan_cloud,
  const std::vector<float>& scan_intensities,
  std::vector<float>& zs,
  Jacobians& Hois,
  Jacobians& Hils)
{
  if (poses_.size() == 0) {
    return;
  }

  const Eigen::Matrix3f R_dpl_dRoi = (state.Til.inverse() * origin_pose_).rotationMatrix();

  const float fx = static_cast<float>(image_width_) / (2.0f * M_PI);
  const float fy = static_cast<float>(image_height_) / vertical_fov_;

  const size_t range = 0;

  // cout << "max_scan_intensity_ = " << max_scan_intensity_ << endl;

  // int scan_step = 1;//scan_points->points.size() / 1000;
  // if (scan_step < 1) {
  //   scan_step = 1;
  // }

  // Htois.reserve(10000);
  // HRois.reserve(10000);
  // Htils.reserve(10000);
  // HRils.reserve(10000);

  // size_t cnt = 0;
  // float e_sum = 0.0f;

  const Sophus::SE3f Tlo = state.Til.inverse() * state.T.inverse();

  if (true) {
    std::vector<std::vector<float>> image(image_width_);
    std::vector<std::vector<float>> depth(image_width_);
    for (size_t u = 0; u < image_width_; ++u) {
      image[u].resize(image_height_);
      depth[u].resize(image_height_);
      for (size_t v = 0; v < image_height_; ++v) {
        image[u][v] = -1.0f;
        depth[u][v] = -1.0f;
      }
    }

    for (size_t i = 0; i < scan_cloud.size(); ++i) {
      const Eigen::Vector3f po = state.T * scan_cloud[i];
      const Eigen::Vector3f pl = Tlo * po;
      Eigen::Vector2i pix;
      point2pixel(pl, pix);
      const int u = pix.x();
      const int v = pix.y();
      if (u < 0 || static_cast<int>(image_width_ - 1) <= u ||
          v < 0 || static_cast<int>(image_height_ - 1) <= v) {
        continue;
      }

      float val = scan_intensities[i] / max_scan_intensity_;
      if (val > 1.0f) {
        val = 1.0f;
      }

      const float d = pl.norm();
      if (depth[u][v] < 0.0f || d < depth[u][v]) {
        image[u][v] = val;
        depth[u][v] = d;
      }
    }

    FILE* fp = fopen("/tmp/local_scan_image.txt", "w");
    for (size_t v = 0; v < image_height_; ++v) {
      for (size_t u = 0; u < image_width_; ++u) {
        fprintf(fp, "%lu %lu %f %f\n", u, v, image[u][v], depth[u][v]);
      }
      fprintf(fp, "\n");
    }
    fclose(fp);
  }

  for (size_t i = 0; i < scan_cloud.size(); ++i) {
    const Eigen::Vector3f po = state.T * scan_cloud[i];
    const Eigen::Vector3f pl = Tlo * po;
    Eigen::Vector2i pix;
    point2pixel(pl, pix);
    const int u = pix.x();
    const int v = pix.y();
    if (u < 1 + static_cast<int>(range) || static_cast<int>(image_width_ - 1 - range) <= u ||
        v < 1 + static_cast<int>(range) || static_cast<int>(image_height_ - 1 - range) <= v) {
      // puts("out of image");
      continue;
    }

    const float x = pl.x();
    const float y = pl.y();
    const float z = pl.z();
    const float x2y2 = x * x + y * y;
    const float L = sqrtf(x2y2) - 0.0f;
    if (L < 0.1f) {
      // puts("too short");
      continue;
    }

    const float R = sqrtf(L * L + z * z);
    const float x2y2_inv = 1.0f / x2y2;
    const float LR2_inv = 1.0f / (L * R * R);
    const float R2_inv = 1.0f / (R * R);

    Eigen::Matrix<float, 2, 3> du_dpl;
    du_dpl << -fx * y * x2y2_inv,     fx * x * x2y2_inv,    0.0f,
              -fy * x * z * LR2_inv, -fy * y * z * LR2_inv, fy * L * R2_inv;

    const float d = pl.norm();
    float val = scan_intensities[i] / max_scan_intensity_;
    if (val > 1.0f) {
      val = 1.0f;
    }

    const Eigen::Matrix3f dpl_dtoi = -R_dpl_dRoi;
    const Eigen::Matrix3f dpl_dRoi = R_dpl_dRoi * Sophus::SO3f::hat(po).matrix();

    for (size_t uu = u - range; uu <= u + range; ++uu) {
      for (size_t vv = v - range; vv <= v + range; ++vv) {
        if (!IsDepthEffective(uu, vv) || scan_image_[uu][vv] < 0.0f) {
          continue;
        }

        // if (fabs(d - depth_map_[uu][vv]) > 10.0f) {
        //   continue;
        // }

        const float w = 1.0f;

        const float e = scan_image_[uu][vv] - val;
        if (fabs(e) > 0.3f) {
          continue;
        }

        const float z = w * e;

        // const float e_norm = fabs(z);
        // if (e_norm > e_max) {
        //   continue;
        // }
        // e_sum += e_norm;
        zs.push_back(z);

        Eigen::Matrix<float, 1, 2> de_du;
        de_du(0, 0) = 0.5f * (scan_image_[uu + 1][vv] - scan_image_[uu - 1][vv]);
        de_du(0, 1) = 0.5f * (scan_image_[uu][vv + 1] - scan_image_[uu][vv - 1]);

        const Eigen::Matrix<float, 1, 3> de_dtoi = de_du * du_dpl * dpl_dtoi;
        const Eigen::Matrix<float, 1, 3> de_dRoi = de_du * du_dpl * dpl_dRoi;
        Eigen::Matrix<float, 1, 6> Hoi;
        Hoi << de_dtoi(0), de_dtoi(1), de_dtoi(2), de_dRoi(0), de_dRoi(1), de_dRoi(2);
        Hois.push_back(w * Hoi);

        Hils.push_back(Eigen::Matrix<float, 1, 6>::Zero());
      }
    }
  }
}

} // namespace pslam