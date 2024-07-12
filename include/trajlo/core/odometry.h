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

#ifndef TRAJLO_ODOMETRY_H
#define TRAJLO_ODOMETRY_H

#include <atomic>
#include <memory>
#include <thread>

#include <tbb/concurrent_queue.h>
#include <Eigen/Eigen>

#include <trajlo/utils/common_type.h>
#include <trajlo/utils/config.h>
#include <trajlo/utils/pose_type.h>
#include <trajlo/utils/eigen_utils.hpp>

#include <trajlo/core/map_manager.h>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace traj {
  class TrajLOdometry : public rclcpp::Node {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Ptr = std::shared_ptr<TrajLOdometry>;

    TrajLOdometry(const TrajConfig& config); // constructor

    ~TrajLOdometry() {
      processing_thread_->join(); // wait for the "processing_thread" to finish
      std::cout << "processing is finished !!!" << std::endl;
    }

    void process();
    void Optimize();
    void Marginalize();
    void PointCloudSegment(Scan::Ptr scan, Measurement::Ptr measure);
    void RangeFilter(Measurement::Ptr measure, std::vector<Eigen::Vector4d>& points);
    void UndistortRawPoints(std::vector<PointXYZIT>& pc_in,
                            std::vector<PointXYZI>& pc_out, const posePair& pp) ;

    tbb::concurrent_bounded_queue<Scan::Ptr> laser_data_queue; // thread-safe queue for laser scan data
    std::atomic<bool> isFinish; // atomic variable for indicating the end of the processing

    tbb::concurrent_bounded_queue<ScanVisData::Ptr>* vis_data_queue = nullptr;

  private:
    MapManager::Ptr map_;
    bool isMove_ = false;

    Sophus::SE3d T_wc_curr; // current pose
    Sophus::SE3d T_prior;  // motion prior

    Eigen::aligned_map<tStampPair, Measurement::Ptr> measurements; // measurements sorted by timestamp pairs
    std::deque<Measurement::Ptr> measure_cache;
    std::vector<PointXYZIT> points_cache;

    double min_range_2_=1.0; // if a point is closer to the origin than this value, it will be discarded
    double converge_thresh_=0.01;

    // pose database
    Eigen::aligned_map<int64_t, PoseStateWithLin<double>> frame_poses_; // poses sorted by timestamp
    using tumPose = std::pair<int64_t, Sophus::SE3d>; // timestamp and pose
    std::vector<tumPose> trajectory_;

    // slide window parameters
    int64_t window_interval_ = 4e7;
    int64_t init_interval_ = 3e8;
    size_t max_frames_ = 4;
    int64_t last_begin_t_ns_;
    int64_t last_end_t_ns_;


    // marginalization
    AbsOrderMap marg_order;
    Eigen::MatrixXd marg_H;
    Eigen::VectorXd marg_b;

    TrajConfig config_;

    // backend
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    std::shared_ptr<std::thread> processing_thread_;
  };

}  // namespace traj

#endif  // TRAJLO_ODOMETRY_H
