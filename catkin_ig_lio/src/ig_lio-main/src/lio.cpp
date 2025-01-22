#include "ig_lio/lio.h"
#include "ig_lio/timer.h"

extern Timer timer;

bool LIO::MeasurementUpdate(SensorMeasurement& sensor_measurement) {
  if (sensor_measurement.measurement_type_ == MeasurementType::LIDAR) {
    // range filter
    CloudPtr filtered_cloud_ptr(new CloudType());
    filtered_cloud_ptr->points.reserve(sensor_measurement.cloud_ptr_->size());
    for (const auto& pt : sensor_measurement.cloud_ptr_->points) {
      if (InRadius(pt)) {
        filtered_cloud_ptr->points.emplace_back(pt);
      }
    }
    sensor_measurement.cloud_ptr_ = filtered_cloud_ptr;

    timer.Evaluate(
        [&, this]() {
          // transform scan from lidar's frame to imu's frame
          CloudPtr cloud_body_ptr(new CloudType());
          pcl::transformPointCloud(*sensor_measurement.cloud_ptr_,
                                   *cloud_body_ptr,
                                   config_.T_imu_lidar);
          sensor_measurement.cloud_ptr_ = std::move(cloud_body_ptr);

          // undistort
          if (config_.enable_undistort) {
            UndistortPointCloud(sensor_measurement.bag_time_,
                                sensor_measurement.lidar_end_time_,
                                sensor_measurement.cloud_ptr_);
          }
        },
        "undistort");

    timer.Evaluate(
        [&, this]() {
          fast_voxel_grid_ptr_->Filter(
              sensor_measurement.cloud_ptr_, cloud_DS_ptr_, cloud_cov_ptr_);
        },
        "downsample");
  }

  // Make sure the local map is dense enought to measurement update
  if (lidar_frame_count_ <= 10) {
    CloudPtr trans_cloud_ptr(new CloudType());
    pcl::transformPointCloud(
        *sensor_measurement.cloud_ptr_, *trans_cloud_ptr, curr_state_.pose);
    voxel_map_ptr_->AddCloud(trans_cloud_ptr);
    lidar_frame_count_++;
    return true;
  }

  // measurement update
  prev_state_ = curr_state_;
  iter_num_ = 0;
  need_converge_ = false;
  Eigen::Matrix<double, 15, 1> delta_x = Eigen::Matrix<double, 15, 1>::Zero();
  while (iter_num_ < config_.max_iterations) {
    StepOptimize(sensor_measurement, delta_x);

    if (IsConverged(delta_x)) {
      // Optimization convergence, exit
      break;
    } else {
      // The first three iterations perform KNN, then no longer perform, thus
      // accelerating the problem convergence
      if (iter_num_ < 3) {
        need_converge_ = false;
      } else {
        need_converge_ = true;
      }
    }

    iter_num_++;
  }

  // LOG(INFO) << "final hessian: " << std::endl << final_hessian_;
  // P_ = final_hessian_.inverse();
  ComputeFinalCovariance(delta_x);
  prev_state_ = curr_state_;

  timer.Evaluate(
      [&, this]() {
        if (lidar_frame_count_ < 10) {
          CloudPtr trans_cloud_ptr(new CloudType());
          pcl::transformPointCloud(*sensor_measurement.cloud_ptr_,
                                   *trans_cloud_ptr,
                                   curr_state_.pose);
          voxel_map_ptr_->AddCloud(trans_cloud_ptr);

          last_keyframe_pose_ = curr_state_.pose;
        } else {
          Eigen::Matrix4d delta_p =
              last_keyframe_pose_.inverse() * curr_state_.pose;
          // The keyframe strategy ensures an appropriate spatial pattern of the
          // points in each voxel
          if (effect_feat_num_ < 1000 ||
              delta_p.block<3, 1>(0, 3).norm() > 0.5 ||
              Sophus::SO3d(delta_p.block<3, 3>(0, 0)).log().norm() > 0.18) {
            CloudPtr trans_cloud_DS_ptr(new CloudType());
            pcl::transformPointCloud(
                *cloud_DS_ptr_, *trans_cloud_DS_ptr, curr_state_.pose);
            voxel_map_ptr_->AddCloud(trans_cloud_DS_ptr);

            last_keyframe_pose_ = curr_state_.pose;
            keyframe_count_++;
          }
        }
      },
      "update voxel map");

  lidar_frame_count_++;

  ava_effect_feat_num_ += (effect_feat_num_ - ava_effect_feat_num_) /
                          static_cast<double>(lidar_frame_count_);
  LOG(INFO) << "curr_feat_num: " << effect_feat_num_
            << " ava_feat_num: " << ava_effect_feat_num_
            << " keyframe_count: " << keyframe_count_
            << " lidar_frame_count: " << lidar_frame_count_
            << " grid_size: " << voxel_map_ptr_->GetVoxelMapSize();
  return true;
}

bool LIO::StepOptimize(const SensorMeasurement& sensor_measurement,
                       Eigen::Matrix<double, 15, 1>& delta_x) {
  Eigen::Matrix<double, 15, 15> H = Eigen::Matrix<double, 15, 15>::Zero();
  Eigen::Matrix<double, 15, 1> b = Eigen::Matrix<double, 15, 1>::Zero();

  double y0 = 0;
  switch (sensor_measurement.measurement_type_) {
  case MeasurementType::LIDAR: {
    double y0_lidar = 0.0;

    timer.Evaluate(
        [&, this]() {
          // After LIO has moved some distance, each voxel is already well
          // formulate
          // the surrounding environments
          if (keyframe_count_ > 20) {
            y0_lidar = ConstructGICPConstraints(H, b);
          }
          // In the initial state, the probability of each voxel is poor
          // use point-to-plane instead of GICP
          else {
            y0_lidar = ConstructPoint2PlaneConstraints(H, b);
          }
        },
        "lidar constraints");

    y0 += y0_lidar;
    break;
  }

  default: {
    LOG(ERROR) << "error measurement type!";
    exit(0);
  }
  }

  // LOG(INFO) << "lidar H: " << std::endl << H << std::endl;

  timer.Evaluate(
      [&, this]() {
        double y0_imu = ConstructImuPriorConstraints(H, b);
        y0 += y0_imu;
      },
      "imu constraint");

  GNStep(sensor_measurement, H, b, y0, delta_x);

  return true;
}

bool LIO::GNStep(const SensorMeasurement& sensor_measurement,
                 Eigen::Matrix<double, 15, 15>& H,
                 Eigen::Matrix<double, 15, 1>& b,
                 const double y0,
                 Eigen::Matrix<double, 15, 1>& delta_x) {
  timer.Evaluate(
      [&, this]() {
        // The function inverse() has better numerical stability
        // And the dimension is small, direct inversion is not time-consuming
        Eigen::Matrix<double, 15, 1> dir = -H.inverse() * b;

        State new_state;
        delta_x = dir;
        CorrectState(curr_state_, delta_x, new_state);
        curr_state_ = new_state;

        final_hessian_ = H;
      },
      "gn step");

  return true;
}

double LIO::ConstructGICPConstraints(Eigen::Matrix<double, 15, 15>& H,
                                     Eigen::Matrix<double, 15, 1>& b) {
  Eigen::Matrix<double, 8, 6> result_matrix =
      Eigen::Matrix<double, 8, 6>::Zero();
  Eigen::Matrix<double, 8, 6> init_matrix = Eigen::Matrix<double, 8, 6>::Zero();
  // if (need_converge_) 的目的是引导程序在不同阶段采用不同的约束处理方式，以提高优化的收敛速度和精度。
  // 在点云配准（GICP）过程中，不同阶段的优化策略可以显著影响计算效率和最终的精度。need_converge_ 变量用于指示当前是否需要更严格的优化约束。通常，配准过程会经历以下两个阶段：
  // 初始阶段（need_converge_ == false）
  // 初始阶段的点云配准可能受外点和较大误差的影响。
  // 此时，点云对应关系尚未准确，误差较大，因此不需要复杂的计算（例如逐点计算）。
  // 这一阶段主要通过简化约束条件（比如点到面约束）来加快计算速度，为后续的精确优化奠定基础。
  // 收敛阶段（need_converge_ == true）
  // 在优化的后期，已经有了较好的初始解，点云间的对应关系较准确。
  // 此时需要精确的约束和更复杂的优化计算（比如加入更多点对，严格计算高斯加权等），以确保最终配准的精度。


  // need_converge_ 的值决定了两种不同的运行逻辑，其核心区别在于
  // 是否需要重新计算匹配关系（Correspondences）。
  // 是否直接利用已有的匹配关系进行优化。
  // 为什么需要两种模式？
  // 初期优化（need_converge_ == false）：
  // 在优化的初期，点云之间的相对位姿误差较大，现有匹配对可能不准确，甚至错误。
  // 因此必须重新进行最近邻搜索，找到更加合理的点对关系，才能保证后续优化的正确性。
  // 后期优化（need_converge_ == true）：
  // 随着优化的进行，点云之间的误差逐渐收敛，匹配点对的关系也趋于稳定。
  // 重新计算匹配点对的收益逐渐降低，而其计算代价很高。
  // 因此，在后期可以直接使用已有的匹配点对，加速优化过程，同时不会显著降低结果精度。
  if (need_converge_) {
    // 这里一共定义了两个 lambda 表达式：
    // 第一个用于处理每个线程的局部结果 local_result。
    // 第二个用于将不同线程的结果进行合并
    result_matrix = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, correspondences_array_.size()),
        init_matrix,
        [&, this](tbb::blocked_range<size_t> r,
                  Eigen::Matrix<double, 8, 6> local_result) {
          for (size_t i = r.begin(); i < r.end(); ++i) {
            Eigen::Vector3d trans_mean_A =
                curr_state_.pose.block<3, 3>(0, 0) *
                    correspondences_array_[i]->mean_A +
                curr_state_.pose.block<3, 1>(0, 3);

            Eigen::Vector3d error =
                correspondences_array_[i]->mean_B - trans_mean_A;

            // without loss function
            // local_result(7, 0) += gicp_constraint_gain_ * error.transpose() *
            //                       correspondences_array_[i]->mahalanobis *
            //                       error;

            // // The residual takes the partial derivative of the state
            // Eigen::Matrix<double, 3, 6> dres_dx =
            //     Eigen::Matrix<double, 3, 6>::Zero();

            // // The residual takes the partial derivative of rotation
            // dres_dx.block<3, 3>(0, 0) =
            //     curr_state_.pose.block<3, 3>(0, 0) *
            //     Sophus::SO3d::hat(correspondences_array_[i]->mean_A);

            // // The residual takes the partial derivative of position
            // dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

            // local_result.block(0, 0, 6, 6) +=
            //     gicp_constraint_gain_ * dres_dx.transpose() *
            //     correspondences_array_[i]->mahalanobis * dres_dx;

            // local_result.block(6, 0, 1, 6) +=
            //     (gicp_constraint_gain_ * dres_dx.transpose() *
            //      correspondences_array_[i]->mahalanobis * error)
            //         .transpose();

            // loss function
            Eigen::Matrix3d mahalanobis =
                correspondences_array_[i]->mahalanobis;
            double cost_function = error.transpose() * mahalanobis * error;
            Eigen::Vector3d rho;
            CauchyLossFunction(cost_function, 10.0, rho);

            local_result(7, 0) += config_.gicp_constraint_gain * rho[0];

            // The residual takes the partial derivative of the state
            Eigen::Matrix<double, 3, 6> dres_dx =
                Eigen::Matrix<double, 3, 6>::Zero();

            // The residual takes the partial derivative of rotation
            //R雅可比
            dres_dx.block<3, 3>(0, 0) =
                curr_state_.pose.block<3, 3>(0, 0) *
                Sophus::SO3d::hat(correspondences_array_[i]->mean_A);

            // The residual takes the partial derivative of position
            //t雅可比
            dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();
            //构造鲁棒信息矩阵 (Robust Information Matrix)，用于加权匹配误差的二次项（Hessian 矩阵）和一阶项（梯度向量）的计算。
            //在鲁棒优化中，通过加入损失函数的导数来调整信息矩阵，从而减弱外点对整体优化的影响。
            // rho[1] 和 rho[2] 是损失函数 ρ(e) 的一阶导数和二阶导数。
            // 如果误差较小（即内点），则ρ[1]≈1，ρ[2]≈0，不会削弱信息矩阵。
            // 如果误差较大（即外点），则 ρ[1]<1，ρ[2]<0，对误差进行削弱。
            Eigen::Matrix3d robust_information_matrix =
                config_.gicp_constraint_gain *
                (rho[1] * mahalanobis + 2.0 * rho[2] * mahalanobis * error *
                                            error.transpose() * mahalanobis);
            //JT * H * J
            local_result.block(0, 0, 6, 6) +=
                dres_dx.transpose() * robust_information_matrix * dres_dx;
            //JT * H * e
            local_result.block(6, 0, 1, 6) +=
                (config_.gicp_constraint_gain * rho[1] * dres_dx.transpose() *
                 mahalanobis * error)
                    .transpose();
          }

          return local_result;
        },
        [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
          return x + y;
        });

    H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
        result_matrix.block<6, 6>(0, 0);
    b.block<6, 1>(IndexErrorOri, 0) +=
        result_matrix.block<1, 6>(6, 0).transpose();

    return result_matrix(7, 0);
  }

  size_t delta_p_size = voxel_map_ptr_->delta_P_.size();
  size_t N = cloud_cov_ptr_->size();
  correspondences_array_.clear();
  result_matrix = tbb::parallel_reduce(
      tbb::blocked_range<size_t>(0, N),
      init_matrix,
      [&, this](tbb::blocked_range<size_t> r,
                Eigen::Matrix<double, 8, 6> local_result) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          const PointCovType& point_cov = cloud_cov_ptr_->points[i];
          const Eigen::Vector3d mean_A =
              point_cov.getVector3fMap().cast<double>();
          const Eigen::Vector3d trans_mean_A =
              curr_state_.pose.block<3, 3>(0, 0) * mean_A +
              curr_state_.pose.block<3, 1>(0, 3);

          Eigen::Matrix3d cov_A;
          cov_A << point_cov.cov[0], point_cov.cov[1], point_cov.cov[2],
              point_cov.cov[1], point_cov.cov[3], point_cov.cov[4],
              point_cov.cov[2], point_cov.cov[4], point_cov.cov[5];


          Eigen::Vector3d mean_B = Eigen::Vector3d::Zero();
          Eigen::Matrix3d cov_B = Eigen::Matrix3d::Zero();

          for (size_t i = 0; i < delta_p_size; ++i) {
            Eigen::Vector3d nearby_point =
                trans_mean_A + voxel_map_ptr_->delta_P_[i];
            size_t hash_idx = voxel_map_ptr_->ComputeHashIndex(nearby_point);
            if (voxel_map_ptr_->GetCentroidAndCovariance(
                    hash_idx, mean_B, cov_B) &&
                voxel_map_ptr_->IsSameGrid(nearby_point, mean_B)) {
              //Eigen::Matrix3d::Identity() * 1e-3：一个小的对角矩阵（对角线元素为 0.001），用于防止协方差矩阵不可逆或病态
              Eigen::Matrix3d mahalanobis =
                  (cov_B +
                   curr_state_.pose.block<3, 3>(0, 0) * cov_A *
                       curr_state_.pose.block<3, 3>(0, 0).transpose() +
                   Eigen::Matrix3d::Identity() * 1e-3)
                      .inverse();

              Eigen::Vector3d error = mean_B - trans_mean_A;
              double chi2_error = error.transpose() * mahalanobis * error;
              if (config_.enable_outlier_rejection) {
                if (iter_num_ > 2 && chi2_error > 7.815) {
                  continue;
                }
              }

              std::shared_ptr<Correspondence> corr_ptr =
                  std::make_shared<Correspondence>();
              corr_ptr->mean_A = mean_A;
              corr_ptr->mean_B = mean_B;
              corr_ptr->mahalanobis = mahalanobis;
              correspondences_array_.emplace_back(corr_ptr);

              // without loss function
              // local_result(7, 0) += gicp_constraint_gain_ * chi2_error;

              // // The residual takes the partial derivative of the state
              // Eigen::Matrix<double, 3, 6> dres_dx =
              //     Eigen::Matrix<double, 3, 6>::Zero();

              // // The residual takes the partial derivative of rotation
              // dres_dx.block<3, 3>(0, 0) = curr_state_.pose.block<3, 3>(0, 0)
              // *
              //                             Sophus::SO3d::hat(mean_A);

              // // The residual takes the partial derivative of position
              // dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

              // local_result.block(0, 0, 6, 6) += gicp_constraint_gain_ *
              //                                   dres_dx.transpose() *
              //                                   mahalanobis * dres_dx;

              // local_result.block(6, 0, 1, 6) +=
              //     (gicp_constraint_gain_ * dres_dx.transpose() * mahalanobis
              //     *
              //      error)
              //         .transpose();

              // loss function
              double cost_function = chi2_error;
              Eigen::Vector3d rho;
              CauchyLossFunction(cost_function, 10.0, rho);

              local_result(7, 0) += config_.gicp_constraint_gain * rho[0];

              // The residual takes the partial derivative of the state
              Eigen::Matrix<double, 3, 6> dres_dx =
                  Eigen::Matrix<double, 3, 6>::Zero();

              // The residual takes the partial derivative of rotation
              dres_dx.block<3, 3>(0, 0) = curr_state_.pose.block<3, 3>(0, 0) *
                                          Sophus::SO3d::hat(mean_A);

              // The residual takes the partial derivative of position
              dres_dx.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

              Eigen::Matrix3d robust_information_matrix =
                  config_.gicp_constraint_gain *
                  (rho[1] * mahalanobis + 2.0 * rho[2] * mahalanobis * error *
                                              error.transpose() * mahalanobis);
              local_result.block(0, 0, 6, 6) +=
                  dres_dx.transpose() * robust_information_matrix * dres_dx;

              local_result.block(6, 0, 1, 6) +=
                  (config_.gicp_constraint_gain * rho[1] * dres_dx.transpose() *
                   mahalanobis * error)
                      .transpose();

              break;
            }
          }
        }

        return local_result;
      },
      [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
        return x + y;
      });

  effect_feat_num_ = correspondences_array_.size();

  H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
      result_matrix.block<6, 6>(0, 0);
  b.block<6, 1>(IndexErrorOri, 0) +=
      result_matrix.block<1, 6>(6, 0).transpose();

  return result_matrix(7, 0);
}

double LIO::ConstructPoint2PlaneConstraints(Eigen::Matrix<double, 15, 15>& H,
                                            Eigen::Matrix<double, 15, 1>& b) {
  Eigen::Matrix<double, 8, 6> result_matrix =
      Eigen::Matrix<double, 8, 6>::Zero();
  Eigen::Matrix<double, 8, 6> init_matrix = Eigen::Matrix<double, 8, 6>::Zero();

  // Skip the KNN to accelerate convergence
  if (need_converge_) {
    result_matrix = tbb::parallel_reduce(
        tbb::blocked_range<size_t>(0, correspondences_array_.size()),
        init_matrix,
        [&, this](tbb::blocked_range<size_t> r,
                  Eigen::Matrix<double, 8, 6> local_result) {
          for (size_t i = r.begin(); i < r.end(); ++i) {
            const Eigen::Vector3d trans_pt =
                curr_state_.pose.block<3, 3>(0, 0) *
                    correspondences_array_[i]->mean_A +
                curr_state_.pose.block<3, 1>(0, 3);
            const Eigen::Vector4d& plane_coeff =
                correspondences_array_[i]->plane_coeff;
            //error = n^T(R * p + t) + d
            double error =
                plane_coeff.head(3).dot(trans_pt) + plane_coeff(3, 0);

            local_result(7, 0) +=
                config_.point2plane_constraint_gain * error * error;

            // The residual takes the partial derivative of the state
            Eigen::Matrix<double, 1, 6> dres_dx =
                Eigen::Matrix<double, 1, 6>::Zero();

            // The residual takes the partial derivative of rotation
            // -n^T * R * p^
            dres_dx.block<1, 3>(0, 0) =
                -plane_coeff.head(3).transpose() *
                curr_state_.pose.block<3, 3>(0, 0) *
                Sophus::SO3d::hat(correspondences_array_[i]->mean_A);

            // The residual takes the partial derivative of position
            // n^T
            dres_dx.block<1, 3>(0, 3) = plane_coeff.head(3).transpose();

            local_result.block(0, 0, 6, 6) +=
                config_.point2plane_constraint_gain * dres_dx.transpose() *
                dres_dx;

            local_result.block(6, 0, 1, 6) +=
                config_.point2plane_constraint_gain * dres_dx * error;
          }

          return local_result;
        },
        [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
          return x + y;
        });

    H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
        result_matrix.block<6, 6>(0, 0);
    b.block<6, 1>(IndexErrorOri, 0) +=
        result_matrix.block<1, 6>(6, 0).transpose();

    return result_matrix(7, 0);
  }

  size_t N = cloud_cov_ptr_->size();
  correspondences_array_.clear();
  result_matrix = tbb::parallel_reduce(
      tbb::blocked_range<size_t>(0, N),
      init_matrix,
      [&, this](tbb::blocked_range<size_t> r,
                Eigen::Matrix<double, 8, 6> local_result) {
        for (size_t i = r.begin(); i < r.end(); ++i) {
          const Eigen::Vector3d p =
              cloud_cov_ptr_->points[i].getVector3fMap().cast<double>();
          const Eigen::Vector3d p_w = curr_state_.pose.block<3, 3>(0, 0) * p +
                                      curr_state_.pose.block<3, 1>(0, 3);

          std::vector<Eigen::Vector3d> nearest_points;
          nearest_points.reserve(10);

          voxel_map_ptr_->KNNByCondition(p_w, 5, 5.0, nearest_points);

          Eigen::Vector4d plane_coeff;
          if (nearest_points.size() >= 3 &&
              EstimatePlane(plane_coeff, nearest_points)) {
            double error = plane_coeff.head(3).dot(p_w) + plane_coeff(3, 0);

            bool is_vaild = p.norm() > (81 * error * error);
            if (is_vaild) {
              std::shared_ptr<Correspondence> corr_ptr =
                  std::make_shared<Correspondence>();
              corr_ptr->mean_A = p;
              corr_ptr->plane_coeff = plane_coeff;
              correspondences_array_.emplace_back(corr_ptr);

              local_result(7, 0) +=
                  config_.point2plane_constraint_gain * error * error;

              // The residual takes the partial derivative of the state
              Eigen::Matrix<double, 1, 6> dres_dx =
                  Eigen::Matrix<double, 1, 6>::Zero();

              // The residual takes the partial derivative of rotation
              dres_dx.block<1, 3>(0, 0) = -plane_coeff.head(3).transpose() *
                                          curr_state_.pose.block<3, 3>(0, 0) *
                                          Sophus::SO3d::hat(p);

              // The residual takes the partial derivative of position
              dres_dx.block<1, 3>(0, 3) = plane_coeff.head(3).transpose();

              local_result.block(0, 0, 6, 6) +=
                  config_.point2plane_constraint_gain * dres_dx.transpose() *
                  dres_dx;

              local_result.block(6, 0, 1, 6) +=
                  config_.point2plane_constraint_gain * dres_dx * error;
            }
          }
        }

        return local_result;
      },
      [](Eigen::Matrix<double, 8, 6> x, Eigen::Matrix<double, 8, 6> y) {
        return x + y;
      });

  effect_feat_num_ = correspondences_array_.size();

  H.block<6, 6>(IndexErrorOri, IndexErrorOri) +=
      result_matrix.block<6, 6>(0, 0);
  b.block<6, 1>(IndexErrorOri, 0) +=
      result_matrix.block<1, 6>(6, 0).transpose();

  return result_matrix(7, 0);
}
// 函数 ConstructImuPriorConstraints 用于构建 IMU 先验约束（IMU Prior Constraint），通过当前状态与上一时刻的状态之间的关系，建立优化问题的 Hessian 矩阵 H 和偏导向量 b
double LIO::ConstructImuPriorConstraints(Eigen::Matrix<double, 15, 15>& H,
                                         Eigen::Matrix<double, 15, 1>& b) {
  Sophus::SO3d ori_diff =
      Sophus::SO3d(prev_state_.pose.block<3, 3>(0, 0).transpose() *
                   curr_state_.pose.block<3, 3>(0, 0));
  Eigen::Vector3d ori_error = ori_diff.log();

  Eigen::Matrix3d right_jacoiban_inv = Sophus::SO3d::jr_inv(ori_diff);

  Eigen::Matrix<double, 15, 15> jacobian =
      Eigen::Matrix<double, 15, 15>::Identity();
  jacobian.block<3, 3>(IndexErrorOri, IndexErrorOri) = right_jacoiban_inv;

  // LOG(INFO) << "imu jacobian: " << std::endl << jacobian;

  Eigen::Matrix<double, 15, 1> residual = Eigen::Matrix<double, 15, 1>::Zero();
  residual.block<3, 1>(IndexErrorOri, 0) = ori_error;
  residual.block<3, 1>(IndexErrorPos, 0) =
      curr_state_.pose.block<3, 1>(0, 3) - prev_state_.pose.block<3, 1>(0, 3);
  residual.block<3, 1>(IndexErrorVel, 0) = curr_state_.vel - prev_state_.vel;
  residual.block<3, 1>(IndexErrorBiasAcc, 0) = curr_state_.ba - prev_state_.ba;
  residual.block<3, 1>(IndexErrorBiasGyr, 0) = curr_state_.bg - prev_state_.bg;

  Eigen::Matrix<double, 15, 15> inv_P = P_.inverse();

  // LOG(INFO) << "inv_P: " << std::endl << inv_P;
  //P 是 IMU 误差状态的协方差矩阵，用于权衡不同误差分量对整体优化目标的贡献。取p^-1是为了计算马氏距离
  H += jacobian.transpose() * inv_P * jacobian;
  b += jacobian.transpose() * inv_P * residual;

  double errors = residual.transpose() * inv_P * residual;

  return errors;
}

bool LIO::Predict(const double time,
                  const Eigen::Vector3d& acc_1,
                  const Eigen::Vector3d& gyr_1) {
  double dt = time - lio_time_;

  Eigen::Vector3d un_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d un_gyr = Eigen::Vector3d::Zero();

  NominalStateUpdate(dt,
                     acc_0_,
                     acc_1,
                     gyr_0_,
                     gyr_1,
                     prev_state_.pose,
                     prev_state_.vel,
                     curr_state_.pose,
                     curr_state_.vel,
                     un_acc,
                     un_gyr);

  ErrorStateUpdate(dt, acc_0_, acc_1, gyr_0_, gyr_1);

  if (config_.enable_undistort) {
    // save the predicted pose for scan undistortion
    PoseHistory ph;
    ph.time_ = time;
    ph.T_ = curr_state_.pose;
    ph.un_acc_ = un_acc;
    ph.un_gyr_ = un_gyr;
    ph.vel_ = curr_state_.vel;
    pose_history_.push_back(ph);
  }

  // save the data for next median integral
  prev_state_ = curr_state_;
  acc_0_ = acc_1;
  gyr_0_ = gyr_1;
  lio_time_ = time;

  return true;
}

bool LIO::NominalStateUpdate(const double dt,
                             const Eigen::Vector3d& acc_0,
                             const Eigen::Vector3d& acc_1,
                             const Eigen::Vector3d& gyr_0,
                             const Eigen::Vector3d& gyr_1,
                             const Eigen::Matrix4d& T_prev,
                             const Eigen::Vector3d& vel_prev,
                             Eigen::Matrix4d& T_curr,
                             Eigen::Vector3d& vel_curr,
                             Eigen::Vector3d& un_acc,
                             Eigen::Vector3d& un_gyr) {
  // update ori
  un_gyr = 0.5 * (gyr_0 + gyr_1) - curr_state_.bg;
  T_curr.block<3, 3>(0, 0) =
      T_prev.block<3, 3>(0, 0) * Sophus::SO3d::exp(un_gyr * dt).matrix();

  Eigen::Vector3d un_acc_0 =
      T_prev.block<3, 3>(0, 0) * (acc_0 - curr_state_.ba);
  Eigen::Vector3d un_acc_1 =
      T_curr.block<3, 3>(0, 0) * (acc_1 - curr_state_.ba);
  un_acc = 0.5 * (un_acc_0 + un_acc_1) - g_;

  // update vel
  vel_curr = vel_prev + un_acc * dt;
  // update pos
  T_curr.block<3, 1>(0, 3) =
      T_prev.block<3, 1>(0, 3) + vel_prev * dt + 0.5 * dt * dt * un_acc;

  return true;
}

bool LIO::ErrorStateUpdate(const double dt,
                           const Eigen::Vector3d& acc_0,
                           const Eigen::Vector3d& acc_1,
                           const Eigen::Vector3d& gyr_0,
                           const Eigen::Vector3d& gyr_1) {
  Eigen::Vector3d w = 0.5 * (gyr_0 + gyr_1) - curr_state_.bg;
  Eigen::Vector3d a0 = acc_0 - curr_state_.ba;
  Eigen::Vector3d a1 = acc_1 - curr_state_.ba;

  Eigen::Matrix3d w_x = Sophus::SO3d::hat(w).matrix();
  Eigen::Matrix3d a0_x = Sophus::SO3d::hat(a0).matrix();
  Eigen::Matrix3d a1_x = Sophus::SO3d::hat(a1).matrix();
  Eigen::Matrix3d I_w_x = Sophus::SO3d::exp(-w * dt).matrix();

  F_.setZero();
  // F_.block<3,3>(IndexErrorVel,IndexErrorOri) =
  //     -0.5 * dt * prev_state_.pose.block<3,3>(0,0) * a0_x
  //     -0.5 * dt * curr_state_.pose.block<3,3>(0,0) * a1_x *
  //     (Eigen::Matrix3d::Identity() - w_x * dt);
  F_.block<3, 3>(IndexErrorVel, IndexErrorOri) =
      -0.5 * dt * prev_state_.pose.block<3, 3>(0, 0) * a0_x -
      0.5 * dt * curr_state_.pose.block<3, 3>(0, 0) * a1_x *
          I_w_x;  // More accurate than above
  F_.block<3, 3>(IndexErrorVel, IndexErrorVel) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(IndexErrorVel, IndexErrorBiasAcc) =
      -0.5 *
      (prev_state_.pose.block<3, 3>(0, 0) +
       curr_state_.pose.block<3, 3>(0, 0)) *
      dt;
  F_.block<3, 3>(IndexErrorVel, IndexErrorBiasGyr) =
      0.5 * curr_state_.pose.block<3, 3>(0, 0) * a1_x * dt * dt;

  F_.block<3, 3>(IndexErrorPos, IndexErrorPos) = Eigen::Matrix3d::Identity();
  F_.block<3, 3>(IndexErrorPos, IndexErrorOri) =
      0.5 * dt * F_.block<3, 3>(IndexErrorVel, IndexErrorOri);
  F_.block<3, 3>(IndexErrorPos, IndexErrorVel) =
      Eigen::Matrix3d::Identity() * dt;
  F_.block<3, 3>(IndexErrorPos, IndexErrorBiasAcc) =
      0.5 * dt * F_.block<3, 3>(IndexErrorVel, IndexErrorBiasAcc);
  F_.block<3, 3>(IndexErrorPos, IndexErrorBiasGyr) =
      0.5 * dt * F_.block<3, 3>(IndexErrorVel, IndexErrorBiasGyr);

  // F_.block<3,3>(IndexErrorOri,IndexErrorOri) = Eigen::Matrix3d::Identity()
  // - w_x * dt;
  F_.block<3, 3>(IndexErrorOri, IndexErrorOri) =
      I_w_x;  // More accurate than above
  F_.block<3, 3>(IndexErrorOri, IndexErrorBiasGyr) =
      -Eigen::Matrix3d::Identity() * dt;

  F_.block<3, 3>(IndexErrorBiasAcc, IndexErrorBiasAcc) =
      Eigen::Matrix3d::Identity();
  F_.block<3, 3>(IndexErrorBiasGyr, IndexErrorBiasGyr) =
      Eigen::Matrix3d::Identity();

  B_.setZero();
  B_.block<3, 3>(IndexErrorVel, IndexNoiseAccLast) =
      0.5 * prev_state_.pose.block<3, 3>(0, 0) * dt;
  B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrLast) =
      -0.25 * curr_state_.pose.block<3, 3>(0, 0) * a1_x * dt * dt;
  B_.block<3, 3>(IndexErrorVel, IndexNoiseAccCurr) =
      0.5 * curr_state_.pose.block<3, 3>(0, 0) * dt;
  B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrCurr) =
      B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrLast);

  B_.block<3, 3>(IndexErrorOri, IndexNoiseGyrLast) =
      0.5 * Eigen::Matrix3d::Identity() * dt;  // inaccuracy
  B_.block<3, 3>(IndexErrorOri, IndexNoiseGyrCurr) =
      B_.block<3, 3>(IndexErrorOri, IndexNoiseGyrLast);

  B_.block<3, 3>(IndexErrorPos, IndexNoiseAccLast) =
      0.5 * B_.block<3, 3>(IndexErrorVel, IndexNoiseAccLast) * dt;
  B_.block<3, 3>(IndexErrorPos, IndexNoiseGyrLast) =
      0.5 * B_.block<3, 3>(IndexErrorVel, IndexNoiseGyrLast) * dt;
  B_.block<3, 3>(IndexErrorPos, IndexNoiseAccCurr) =
      0.5 * B_.block<3, 3>(IndexErrorVel, IndexNoiseAccCurr) * dt;
  B_.block<3, 3>(IndexErrorPos, IndexNoiseGyrCurr) =
      B_.block<3, 3>(IndexErrorPos, IndexNoiseGyrLast);

  B_.block<3, 3>(IndexErrorBiasAcc, IndexNoiseBiasAcc) =
      Eigen::Matrix3d::Identity() * dt;
  B_.block<3, 3>(IndexErrorBiasGyr, IndexNoiseBiasGyr) =
      B_.block<3, 3>(IndexErrorBiasAcc, IndexNoiseBiasAcc);

  P_ = F_ * P_ * F_.transpose() + B_ * Q_ * B_.transpose();
  return true;
}

// Undistortion based on median integral
bool LIO::UndistortPointCloud(const double bag_time,
                              const double lidar_end_time,
                              CloudPtr& cloud_ptr) {
  Eigen::Matrix3d R_w_be = curr_state_.pose.block<3, 3>(0, 0);
  Eigen::Vector3d t_w_be = curr_state_.pose.block<3, 1>(0, 3);
  auto it_pt = cloud_ptr->points.end() - 1;
  bool finshed_flag = false;
  for (auto it_pose = pose_history_.end() - 1; it_pose != pose_history_.begin();
       --it_pose) {
    auto bi = it_pose - 1;
    auto bj = it_pose;
    Eigen::Matrix3d R_w_bi = bi->T_.block<3, 3>(0, 0);
    Eigen::Vector3d t_w_bi = bi->T_.block<3, 1>(0, 3);
    Eigen::Vector3d v_w_bi = bi->vel_;
    Eigen::Vector3d un_acc_bj = bj->un_acc_;
    Eigen::Vector3d un_gyr_bj = bj->un_gyr_;

    for (; (it_pt->curvature / (double)(1000) + bag_time) > bi->time_;
         --it_pt) {
      double dt = (it_pt->curvature / (double)(1000) + bag_time) - bi->time_;

      Eigen::Matrix3d R_w_bk =
          R_w_bi * Sophus::SO3d::exp(un_gyr_bj * dt).matrix();
      Eigen::Vector3d t_w_bebk =
          t_w_bi + v_w_bi * dt + 0.5 * dt * dt * un_acc_bj - t_w_be;
      // point_K
      Eigen::Vector3d P_bk_bkK(it_pt->x, it_pt->y, it_pt->z);
      Eigen::Vector3d P_w_beK =
          R_w_be.transpose() * (R_w_bk * P_bk_bkK + t_w_bebk);

      it_pt->x = P_w_beK.x();
      it_pt->y = P_w_beK.y();
      it_pt->z = P_w_beK.z();

      if (it_pt == cloud_ptr->points.begin()) {
        finshed_flag = true;
        break;
      }
    }

    if (finshed_flag) {
      break;
    }
  }

  // Remove excess history imu_pose
  while (!pose_history_.empty() &&
         (pose_history_.front().time_ < lidar_end_time)) {
    pose_history_.pop_front();
  }

  return true;
}

bool LIO::StaticInitialization(SensorMeasurement& sensor_measurement) {
  if (first_imu_frame_) {
    const auto& acc = sensor_measurement.imu_buff_.front().linear_acceleration;
    const auto& gyr = sensor_measurement.imu_buff_.front().angular_velocity;
    imu_init_buff_.emplace_back(Eigen::Vector3d(acc.x, acc.y, acc.z),
                                Eigen::Vector3d(gyr.x, gyr.y, gyr.z));
  }

  for (const auto& imu_msg : sensor_measurement.imu_buff_) {
    Eigen::Vector3d acc(imu_msg.linear_acceleration.x,
                        imu_msg.linear_acceleration.y,
                        imu_msg.linear_acceleration.z);
    Eigen::Vector3d gyr(imu_msg.angular_velocity.x,
                        imu_msg.angular_velocity.y,
                        imu_msg.angular_velocity.z);

    imu_init_buff_.emplace_back(acc, gyr);
  }

  if (imu_init_buff_.size() < max_init_count_) {
    return false;
  }

  Eigen::Vector3d acc_cov, gyr_cov;
  ComputeMeanAndCovDiag(
      imu_init_buff_,
      mean_acc_,
      acc_cov,
      [](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
        return imu_data.first;
      });
  ComputeMeanAndCovDiag(
      imu_init_buff_,
      mean_gyr_,
      gyr_cov,
      [](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
        return imu_data.second;
      });

  // Compute initial attitude via Schmidt orthogonalization.
  // The roll and pitch are aligned with the direction of gravity, but the yaw
  // is random.
  //mean_acc_ 是经过计算得到的加速度的均值向量。为了保证这个向量是单位向量，使用 normalized() 方法将其标准化，
  //从而得到一个单位向量 z_axis。这个向量指示的是加速度的方向，也就是重力的方向（假设设备在静止或者低速运动时，IMU 传感器主要感知的是重力加速度）。
  Eigen::Vector3d z_axis = mean_acc_.normalized();
  //为了构建一个正交的坐标系，我们需要计算一个与重力方向 z_axis 正交的向量。我们选择了一个标准的单位向量 e1 = (1, 0, 0)，它是 X 轴的单位向量。默认imu在水平面上重力垂直向下
  Eigen::Vector3d e1(1, 0, 0);
//   z_axis * z_axis.transpose() 会得到一个 矩阵，这个矩阵可以看作是一个投影矩阵，它用于将任何向量投影到 z_axis 上。
//   z_axis.transpose() * e1 计算的是 e1 在 z_axis 方向上的分量，即 e1 在 z_axis 上的投影长度。
//   然后，通过 z_axis * z_axis.transpose() * e1 计算得到 e1 在 z_axis 方向上的投影向量。
//   简单来说，这一项是在去除 e1 向量中与 z_axis 平行的部分，保留其与 z_axis 垂直的部分。
//   为什么要做投影？
//   假设设备的加速度传感器安装在一个不完全水平的平面上，mean_acc_ 向量会指向地球的重力加速度方向。
//   此时 e1 作为初始的 X 轴参考向量，可能会与 z_axis 不完全垂直。因此，我们需要通过投影的方式来 去除 e1 向量中与重力方向 z_axis 平行的部分，
//   使得剩下的 x_axis 与 z_axis 保持正交。这样，我们最终得到的 x_axis 才是纯粹的水平向量
  //这段代码的作用是通过去除重力方向的分量，计算出与 z_axis 正交的 x_axis
  Eigen::Vector3d x_axis = e1 - z_axis * z_axis.transpose() * e1;
  x_axis.normalize();
  //得到 x_axis 后，我们可以通过叉积得到与 x_axis 和 z_axis 正交的 y_axis，即设备的 Y 轴方
  Eigen::Vector3d y_axis = Sophus::SO3d::hat(z_axis).matrix() * x_axis;
  y_axis.normalize();

  Eigen::Matrix3d init_R;
  init_R.block<3, 1>(0, 0) = x_axis;
  init_R.block<3, 1>(0, 1) = y_axis;
  init_R.block<3, 1>(0, 2) = z_axis;
  Eigen::Quaterniond init_q(init_R);
  curr_state_.pose.block<3, 3>(0, 0) =
      init_q.normalized().toRotationMatrix().transpose();
  //计算 IMU 初始加速度偏差（ba）
  //imu_data.first 是 IMU 测量的加速度。
  //curr_state_.pose.block<3, 3>(0, 0).transpose() * g_ 是将当前 IMU 姿态下的重力加速度转换到世界坐标系中的加速度向量。
  //temp_ba 计算的是当前时刻测得的加速度与重力加速度之间的差值，这个差值就是 加速度偏差，即 ba。
  Eigen::Vector3d init_ba = Eigen::Vector3d::Zero();
  ComputeMeanAndCovDiag(
      imu_init_buff_,
      init_ba,
      acc_cov,
      [this](const std::pair<Eigen::Vector3d, Eigen::Vector3d>& imu_data) {
        Eigen::Vector3d temp_ba =
            imu_data.first -
            curr_state_.pose.block<3, 3>(0, 0).transpose() * g_;
        return temp_ba;
      });

  // init pose
  curr_state_.pose.block<3, 1>(0, 3).setZero();
  // init velocity
  curr_state_.vel.setZero();
  // init bg
  curr_state_.bg = mean_gyr_;
  // init ba
  curr_state_.ba = init_ba;

  prev_state_ = curr_state_;
  //初始化状态协方差矩阵P和过程噪声协方差矩阵Q P 15*15矩阵 Q 18*18矩阵
  //P 是 状态估计的协方差矩阵，它描述了系统状态估计的不确定性。每个状态变量（如位置、速度、姿态、偏差等）都有一个不确定性的度量，而矩阵 P 由多个块组成，表示不同状态变量之间的相关性。
  P_.setIdentity();
  P_.block<3, 3>(IndexErrorOri, IndexErrorOri) =
      config_.init_ori_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorPos, IndexErrorPos) =
      config_.init_pos_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorVel, IndexErrorVel) =
      config_.init_vel_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasAcc, IndexErrorBiasAcc) =
      config_.init_ba_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasGyr, IndexErrorBiasGyr) =
      config_.init_bg_cov * Eigen::Matrix3d::Identity();
  //Q 是 过程噪声协方差矩阵，表示系统中各个状态变量在动态更新过程中的噪声。与 P 的不同之处在于，Q 描述的是由外部扰动、传感器噪声等引起的过程噪声，它定义了系统模型的不确定性
  //加速度计噪声 (Acc Noise): 可能分为前一个时刻（IndexNoiseAccLast）和当前时刻（IndexNoiseAccCurr）的噪声。
  //陀螺仪噪声 (Gyr Noise): 可能分为前一个时刻（IndexNoiseGyrLast）和当前时刻（IndexNoiseGyrCurr）的噪声。
  //加速度计偏差噪声 (Bias Acc Noise): 对应加速度计偏差的过程噪声。
  //陀螺仪偏差噪声 (Bias Gyr Noise): 对应陀螺仪偏差的过程噪声。
  Q_.setIdentity();
  Q_.block<3, 3>(IndexNoiseAccLast, IndexNoiseAccLast) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrLast, IndexNoiseGyrLast) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseAccCurr, IndexNoiseAccCurr) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrCurr, IndexNoiseGyrCurr) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasAcc, IndexNoiseBiasAcc) =
      config_.ba_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasGyr, IndexNoiseBiasGyr) =
      config_.bg_cov * Eigen::Matrix3d::Identity();

  lio_time_ = sensor_measurement.imu_buff_.back().header.stamp.toSec();
  lio_init_ = true;

  LOG(INFO) << "imu static, mean_acc_: " << mean_acc_.transpose()
            << " init_ba: " << init_ba.transpose() << ", ori: " << std::endl
            << curr_state_.pose.block<3, 3>(0, 0);

  return true;
}

bool LIO::AHRSInitialization(SensorMeasurement& sensor_measurement) {
  const auto& back_imu = sensor_measurement.imu_buff_.back();

  if ((back_imu.orientation.w * back_imu.orientation.w +
       back_imu.orientation.x * back_imu.orientation.x +
       back_imu.orientation.y * back_imu.orientation.y +
       back_imu.orientation.z * back_imu.orientation.z) < 1.0) {
    LOG(ERROR) << "AHRS initalization falid, please use static initalizaiton!";
    return false;
  }

  Eigen::Quaterniond temp_q(back_imu.orientation.w,
                            back_imu.orientation.x,
                            back_imu.orientation.y,
                            back_imu.orientation.z);

  curr_state_.pose.block<3, 3>(0, 0) = temp_q.toRotationMatrix();

  curr_state_.pose.block<3, 1>(0, 3).setZero();

  curr_state_.vel.setZero();

  curr_state_.bg.setZero();

  curr_state_.ba.setZero();

  prev_state_ = curr_state_;
//   init_ori_cov：表示初始姿态的不确定性。
//   init_pos_cov：表示初始位置的不确定性。
//   init_vel_cov：表示初始速度的不确定性。
//   init_ba_cov：表示加速度计偏差的不确定性。
//   init_bg_cov：表示陀螺仪偏差的不确定性。
  P_.setIdentity();
  P_.block<3, 3>(IndexErrorOri, IndexErrorOri) =
      config_.init_ori_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorPos, IndexErrorPos) =
      config_.init_pos_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorVel, IndexErrorVel) =
      config_.init_vel_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasAcc, IndexErrorBiasAcc) =
      config_.init_ba_cov * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(IndexErrorBiasGyr, IndexErrorBiasGyr) =
      config_.init_bg_cov * Eigen::Matrix3d::Identity();

  Q_.setIdentity();
  Q_.block<3, 3>(IndexNoiseAccLast, IndexNoiseAccLast) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrLast, IndexNoiseGyrLast) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseAccCurr, IndexNoiseAccCurr) =
      config_.acc_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseGyrCurr, IndexNoiseGyrCurr) =
      config_.gyr_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasAcc, IndexNoiseBiasAcc) =
      config_.ba_cov * Eigen::Matrix3d::Identity();
  Q_.block<3, 3>(IndexNoiseBiasGyr, IndexNoiseBiasGyr) =
      config_.bg_cov * Eigen::Matrix3d::Identity();

  lio_time_ = sensor_measurement.imu_buff_.back().header.stamp.toSec();
  lio_init_ = true;

  return true;
}

bool LIO::CorrectState(const State& state,
                       const Eigen::Matrix<double, 15, 1>& delta_x,
                       State& corrected_state) {
  // ori
  Eigen::Matrix3d delta_R =
      Sophus::SO3d::exp(delta_x.block<3, 1>(IndexErrorOri, 0)).matrix();
  // The normalization is employed after each update to pervent numerical
  // stability
  Eigen::Quaterniond temp_q(state.pose.block<3, 3>(0, 0) * delta_R);
  temp_q.normalize();
  corrected_state.pose.block<3, 3>(0, 0) = temp_q.toRotationMatrix();
  // pos
  corrected_state.pose.block<3, 1>(0, 3) =
      state.pose.block<3, 1>(0, 3) + delta_x.block<3, 1>(IndexErrorPos, 0);
  // vel
  corrected_state.vel = state.vel + delta_x.block<3, 1>(IndexErrorVel, 0);
  // ba
  corrected_state.ba = state.ba + delta_x.block<3, 1>(IndexErrorBiasAcc, 0);
  // bg
  corrected_state.bg = state.bg + delta_x.block<3, 1>(IndexErrorBiasGyr, 0);




  return true;
}
//该函数计算最终状态的协方差矩阵P，结合更新量δx对协方差进行投影变换，保证数值稳定性并校正旋转部分的非线性误差。
bool LIO::ComputeFinalCovariance(const Eigen::Matrix<double, 15, 1>& delta_x) {
  Eigen::Matrix<double, 15, 15> temp_P = final_hessian_.inverse();

  // project covariance
  Eigen::Matrix<double, 15, 15> L = Eigen::Matrix<double, 15, 15>::Identity();
  L.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() -
                        0.5 * Sophus::SO3d::hat(delta_x.block<3, 1>(0, 0));
  P_ = L * temp_P * L;

  return true;
}

bool LIO::IsConverged(const Eigen::Matrix<double, 15, 1>& delta_x) {
  return delta_x.lpNorm<Eigen::Infinity>() < transformation_epsilon_;
}
