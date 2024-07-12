/**
MIT License

Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <trajlo/core/odometry.h>
#include <trajlo/utils/sophus_utils.hpp>

#include <fstream>  
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
namespace traj {
TrajLOdometry::TrajLOdometry(const TrajConfig& config) : rclcpp::Node("trajlo"), config_(config), isFinish(false) {
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("trajlo_pose", 10);

  min_range_2_ = config.min_range * config.min_range; // square of the minimum range
  converge_thresh_ = config.converge_thresh;

  laser_data_queue.set_capacity(100);

  init_interval_ = config.init_interval;
  window_interval_ = config.seg_interval;
  max_frames_ = config.seg_num;

  map_.reset(new MapManager(config.ds_size, config.voxel_size,
                            config.planer_thresh, config.max_voxel_num,
                            config.max_range));

  // setup marginalization
  marg_H.setZero(POSE_SIZE, POSE_SIZE);
  marg_b.setZero(POSE_SIZE);
  double init_pose_weight = config.init_pose_weight;
  marg_H.diagonal().setConstant(init_pose_weight);

  processing_thread_.reset(new std::thread(&TrajLOdometry::process, this));
}

void TrajLOdometry::process() {
  std::cout << "Starting LiDAR Odometry process()" << std::endl;
  int frame_id = 0;
  Scan::Ptr curr_scan;
  bool first_scan = true;
  Measurement::Ptr measure;

  while (rclcpp::ok()) {
    
    // pop operation will block until the valid scan coming 
    laser_data_queue.pop(curr_scan); // get the scan data from the queue and assign it to "curr_scan"
    if (!curr_scan.get()) break; // if the scan data is empty, break the loop
    if (first_scan) {
      last_begin_t_ns_ = curr_scan->timestamp;

      // the first init_interval amount of time is used to initialize the map
      last_end_t_ns_ = last_begin_t_ns_ + init_interval_;
      first_scan = false;
    }
    PointCloudSegment(curr_scan, measure); // segment the point cloud scan into measurements and update the "measure_cache" variable
    while (!measure_cache.empty()) {

      // get measurement
      measure = measure_cache.front();
      measure_cache.pop_front();

      // 1. range filter & compute the relative timestamps
      std::vector<Eigen::Vector4d> points;
      
      // filter the points in the measurement (whether they are too close or too far away) and update the "points" variable
      RangeFilter(measure, points); 

      const auto& tp = measure->tp; // start and end timestamps of the measurement

      if (!map_->IsInit()) {
        // initially set the current pose to the identity transformation (no rotation or translation)
        T_wc_curr = Sophus::SE3d(); // This line effectively resets T_wc_curr to an identity transformation, indicating no rotation or translation.
        map_->MapInit(points);

        // standing start
        frame_poses_[tp.second] = PoseStateWithLin<double>(tp.second, T_wc_curr, true); // set the current pose to the identity transformation, with linearized=true
        trajectory_.emplace_back(tp.first, T_wc_curr); // add the current pose to the trajectory
        map_->SetInit();

        T_prior = Sophus::SE3d(); // set the prior transformation to the identity transformation
      } else {
        measure->pseudoPrior = T_prior;
        measurements[tp] = measure; // add the measurement to the measurements map, which is sorted by timestamp pairs
        
        Sophus::SE3d T_w_pred = frame_poses_[tp.first].getPose() * T_prior; // predict the pose based on the prior transformation
        frame_poses_[tp.second] = PoseStateWithLin<double>(tp.second, T_w_pred); // save the predicted pose

        // 2. preprocess the point cloud.
        map_->PreProcess(points, tp); // downsample the points to reduce the number of points in the point cloud

        if (!isMove_) {
          isMove_ = (T_w_pred).translation().norm() > 0.5; // whether it's a significant movement
        } else {
          map_->ComputeThreshold(); // compute the registration threshold based on the accumulated model errors
        }

        // 3. find the optimal control poses based on the geometric and motion constrains
        // The Optimize function performs non-linear optimization to refine the estimated trajectory 
        // based on geometric and motion constraints.
        Optimize();

        T_wc_curr = frame_poses_[tp.second].getPose(); // update the current pose
        Sophus::SE3d model_deviation = T_w_pred.inverse() * T_wc_curr; // deviation is difference between predicted and current poses
        map_->UpdateModelDeviation(model_deviation);

        // This line updates the prior transformation (T_prior) by calculating the relative transformation 
        // between the previous pose (frame_poses_[tp.first].getPose()) and the 
        // current pose (frame_poses_[tp.second].getPose()).
        T_prior = frame_poses_[tp.first].getPose().inverse() * frame_poses_[tp.second].getPose();

        // Save current pose to the trajectory
        int64_t timestamp = tp.second;
        auto position = T_wc_curr.translation();
        auto orientation = T_wc_curr.unit_quaternion();

        int32_t seconds = static_cast<int32_t>(timestamp / 1e9);
        uint32_t nanoseconds = static_cast<uint32_t>(timestamp % static_cast<int64_t>(1e9));

        auto message = geometry_msgs::msg::PoseStamped();
        message.header.stamp.sec = seconds;
        message.header.stamp.nanosec = nanoseconds;
        message.header.frame_id = "world";

        message.pose.position.x = position.x();
        message.pose.position.y = position.y();
        message.pose.position.z = position.z();

        // Set the orientation
        message.pose.orientation.x = orientation.x();
        message.pose.orientation.y = orientation.y();
        message.pose.orientation.z = orientation.z();
        message.pose.orientation.w = orientation.w();

        // Publish the message
        publisher_->publish(message);

        // 4. marginalize the oldest segment and update the map using points
        // belonging to the oldest segment.
        Marginalize();
      }
      frame_id++;
    }
  }
  isFinish = true;
  std::cout << "Finisher LiDAR Odometry " << std::endl;
}


/*
 * The analytic Jacobians in the paper are derived in SE(3) form. For the
 * efficiency in our implementation, we instead update poses in SO(3)+R3
 * form. The connection between them has been discussed in
 * https://gitlab.com/VladyslavUsenko/basalt/-/issues/37
 * */
void TrajLOdometry::Optimize() {
  AbsOrderMap aom;

  // populate the absolute order map with the poses from frame_poses_
  for (const auto& kv : frame_poses_) {
    aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, POSE_SIZE);
    aom.total_size += POSE_SIZE;
    aom.items++;
  }

  Eigen::MatrixXd abs_H; // Hessian matrix
  Eigen::VectorXd abs_b; // gradient

  // iterative optimization
  for (int iter = 0; iter < config_.max_iterations; iter++) {
    abs_H.setZero(aom.total_size, aom.total_size);
    abs_b.setZero(aom.total_size);

    // 两帧优化: "Two-frame optimization"
    for (auto& m : measurements) {
      int64_t idx_prev = m.first.first;
      int64_t idx_curr = m.first.second;

      const auto& prev = frame_poses_[idx_prev];
      const auto& curr = frame_poses_[idx_curr];

      posePair pp{prev.getPose(), curr.getPose()};
      const tStampPair& tp = m.second->tp;  //{idx_prev,idx_curr};

      // 1. Geometric constrains from lidar point cloud.
      map_->PointRegistrationNormal({prev, curr}, tp, m.second->delta_H,
                                    m.second->delta_b, m.second->lastError,
                                    m.second->lastInliers);

      // 2. Motion constrains behind continuous movement.
      // Log(Tbe)-Log(prior) Equ.(6)
      {
        Sophus::SE3d T_be = pp.first.inverse() * pp.second;
        Sophus::Vector6d tau = Sophus::se3_logd(T_be);
        Sophus::Vector6d res = tau - Sophus::se3_logd(m.second->pseudoPrior);

        Sophus::Matrix6d J_T_w_b;
        Sophus::Matrix6d J_T_w_e;
        Sophus::Matrix6d rr_b;
        Sophus::Matrix6d rr_e;

        if (prev.isLinearized() || curr.isLinearized()) {
          pp = std::make_pair(prev.getPoseLin(), curr.getPoseLin());
          T_be = pp.first.inverse() * pp.second;
          tau = Sophus::se3_logd(T_be);
        }

        Sophus::rightJacobianInvSE3Decoupled(tau, J_T_w_e);
        J_T_w_b = -J_T_w_e * (T_be.inverse()).Adj();

        rr_b.setIdentity();
        rr_b.topLeftCorner<3, 3>() = pp.first.rotationMatrix().transpose();
        rr_e.setIdentity();
        rr_e.topLeftCorner<3, 3>() = pp.second.rotationMatrix().transpose();

        Eigen::Matrix<double, 6, 12> J_be;
        J_be.topLeftCorner<6, 6>() = J_T_w_b * rr_b;
        J_be.topRightCorner<6, 6>() = J_T_w_e * rr_e;

        double alpha_e = config_.kinematic_constrain * m.second->lastInliers;
        m.second->delta_H += alpha_e * J_be.transpose() * J_be;
        m.second->delta_b -= alpha_e * J_be.transpose() * res;
      }

      int abs_id = aom.abs_order_map.at(idx_prev).first;
      abs_H.block<POSE_SIZE * 2, POSE_SIZE * 2>(abs_id, abs_id) +=
          m.second->delta_H;
      abs_b.segment<POSE_SIZE * 2>(abs_id) += m.second->delta_b;
    }

    // Marginalization Error Term
    // reference: Square Root Marginalization for Sliding-Window Bundle
    // Adjustment (N Demmel, D Schubert, C Sommer, D Cremers and V Usenko)
    // https://arxiv.org/abs/2109.02182
    Eigen::VectorXd delta;
    for (const auto& p : frame_poses_) {
      if (p.second.isLinearized()) {
        delta = p.second.getDelta();
      }
    }
    abs_H.block<POSE_SIZE, POSE_SIZE>(0, 0) += marg_H;
    abs_b.head<POSE_SIZE>() -= marg_b;
    abs_b.head<POSE_SIZE>() -= (marg_H * delta);

    Eigen::VectorXd update = abs_H.ldlt().solve(abs_b);
    double max_inc = update.array().abs().maxCoeff();

    if (max_inc < converge_thresh_) {
      break;
    }

    for (auto& kv : frame_poses_) {
      int idx = aom.abs_order_map.at(kv.first).first;
      kv.second.applyInc(update.segment<POSE_SIZE>(idx));
    }
  }

  // update pseudo motion prior after each optimization
  int64_t begin_t = measurements.begin()->first.first;
  int64_t end_t = measurements.begin()->first.second;
  auto begin = frame_poses_[begin_t];
  auto end = frame_poses_[end_t];

  const int64_t m0_t = begin_t;
  for (auto m : measurements) {
    if (m.first.first == m0_t) continue;
    m.second->pseudoPrior = begin.getPose().inverse() * end.getPose();
    begin = frame_poses_[m.first.first];
    end = frame_poses_[m.first.second];
  }
}

void TrajLOdometry::Marginalize() {
  // remove pose with minimal timestamp
  if (measurements.size() >= max_frames_) {
    const auto& tp = measurements.begin()->first;
    const posePair pp{frame_poses_[tp.first].getPose(),
                      frame_poses_[tp.second].getPose()};
    map_->Update(pp, tp);

    Eigen::VectorXd delta = frame_poses_[tp.first].getDelta();

    Eigen::Matrix<double, 12, 12> marg_H_new =
        measurements.begin()->second->delta_H;
    Eigen::Matrix<double, 12, 1> marg_b_new =
        measurements.begin()->second->delta_b;
    marg_H_new.topLeftCorner<POSE_SIZE, POSE_SIZE>() += marg_H;

    marg_b_new.head<POSE_SIZE>() -= marg_b;
    marg_b_new.head<POSE_SIZE>() -= (marg_H * delta);

    Eigen::MatrixXd H_mm_inv =
        marg_H_new.topLeftCorner<6, 6>().fullPivLu().solve(
            Eigen::MatrixXd::Identity(6, 6));
    marg_H_new.bottomLeftCorner<6, 6>() *= H_mm_inv;

    marg_H = marg_H_new.bottomRightCorner<6, 6>();
    marg_b = marg_b_new.tail<6>();
    marg_H -=
        marg_H_new.bottomLeftCorner<6, 6>() * marg_H_new.topRightCorner<6, 6>();
    marg_b -= marg_H_new.bottomLeftCorner<6, 6>() * marg_b_new.head<6>();


    // erase
    frame_poses_.erase(tp.first);
    measurements.erase(tp);

    trajectory_.emplace_back(tp.first, pp.first);
    frame_poses_[tp.second].setLinTrue();
  }
}

/* 
* The function segments a point cloud into measurements based on the timestamps of the points.
* Each measurement encompasses points within a specific time window.
* measurements are stored in the "measure_cache" variable.
*/
void TrajLOdometry::PointCloudSegment(Scan::Ptr scan,
                                      Measurement::Ptr measure) {

  for (size_t i = 0; i < scan->size; i++) { // for all points in the scan
    const auto& p = scan->points[i]; // p: point; X, Y, Z, intensity, timestamp
    if (static_cast<int64_t>(p.ts * 1e9) < last_end_t_ns_) { // if we are still in the same segment
      points_cache.emplace_back(p); // add the point to the cache
    } else { // if we are in a new segment
      // pub one measurement
      measure.reset(new Measurement);
      measure->tp = {last_begin_t_ns_, last_end_t_ns_};
      measure->points = points_cache;
      measure_cache.push_back(measure); // save the points in the current segment to the cache

      // reset cache and time
      points_cache.clear();

      // update time intervals for the next segment
      last_begin_t_ns_ = last_end_t_ns_;
      last_end_t_ns_ = last_begin_t_ns_ + window_interval_;

      if (static_cast<int64_t>(p.ts * 1e9) < last_end_t_ns_) {
        points_cache.emplace_back(p);
      }
    }
  }
}

void TrajLOdometry::RangeFilter(Measurement::Ptr measure,
                                std::vector<Eigen::Vector4d>& points) {
  points.reserve(measure->points.size());
  const auto& tp = measure->tp;
  
  double interv = (tp.second - tp.first) * 1e-9; // interval in which the points were collected, in seconds
  double begin_ts = tp.first * 1e-9; // timestamp of the first point in the measurement, in seconds

  for (const auto& p : measure->points) { // for each point in measurement
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
    double len = (p.x * p.x + p.y * p.y + p.z * p.z); // square of distance from the origin
    if (len < min_range_2_) continue; // if the point is too close to the origin, skip it

    double alpha = (p.ts - begin_ts) / interv; // normalized (relative) timestamp of the point
    
    points.emplace_back(Eigen::Vector4d(p.x, p.y, p.z, alpha)); // add the point to the list of filtered points
  }
}

void TrajLOdometry::UndistortRawPoints(std::vector<PointXYZIT>& pc_in,
                                       std::vector<PointXYZI>& pc_out,
                                       const posePair& pp) {
  Sophus::Vector6f tau =
      Sophus::se3_logd(pp.first.inverse() * pp.second).cast<float>();

  double interv = (pc_in.back().ts - pc_in.front().ts);
  double begin_ts = pc_in.front().ts;

  pc_out.reserve(pc_in.size());
  int i = 0;
  for (const auto& p : pc_in) {
    if (i % config_.point_num == 0) {
      //        float alpha = i * 1.0f / num;
      float alpha = (p.ts - begin_ts) / interv;
      Eigen::Vector3f point(p.x, p.y, p.z);
      if (point.hasNaN() || point.squaredNorm() < 4) continue;

      Sophus::SE3f T_b_i = Sophus::se3_expd(alpha * tau);
      point = T_b_i * point;
      PointXYZI po{point(0), point(1), point(2), p.intensity};
      pc_out.emplace_back(po);
    }
    i++;
  }
}

}  // namespace traj
