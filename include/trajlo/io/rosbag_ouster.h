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

#ifndef TRAJLO_ROSBAG_OUSTER_H
#define TRAJLO_ROSBAG_OUSTER_H

#include <trajlo/io/data_loader.h>

#include <algorithm>
#include <vector>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.h>

namespace traj {
class RosbagOuster : public DataLoader {
 public:
  RosbagOuster() : DataLoader("rosbag_ouster_node"), n(0) {};
  ~RosbagOuster() override = default;

  void publish(const std::string &path, const std::string &topic) override {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, 10, std::bind(&RosbagOuster::topic_callback, this, std::placeholders::_1));
    
      RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
  }

 private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!is_pub_) return;

    Scan::Ptr scan(new Scan);
    scan->timestamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

    int num = msg->height * msg->width;
    int x_idx = getPointCloud2FieldIndex(*msg, "x");
    int y_idx = getPointCloud2FieldIndex(*msg, "y");
    int z_idx = getPointCloud2FieldIndex(*msg, "z");
    int i_idx = getPointCloud2FieldIndex(*msg, "intensity");
    int ts_idx = getPointCloud2FieldIndex(*msg, "t");

    int x_offset = msg->fields[x_idx].offset;
    int y_offset = msg->fields[y_idx].offset;
    int z_offset = msg->fields[z_idx].offset;
    int i_offset = msg->fields[i_idx].offset;
    int ts_offset = msg->fields[ts_idx].offset;

    int step = msg->point_step;

    double tbase = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9; // => çöküyor

    float x, y, z, intensity;
    double ts;
    for (size_t j = 0; j < num; ++j) {
      x = sensor_msgs::readPointCloud2BufferValue<float>(&msg->data[j * step + x_offset], msg->fields[x_idx].datatype);
      y = sensor_msgs::readPointCloud2BufferValue<float>(&msg->data[j * step + y_offset], msg->fields[y_idx].datatype);
      z = sensor_msgs::readPointCloud2BufferValue<float>(&msg->data[j * step + z_offset], msg->fields[z_idx].datatype);
      intensity = sensor_msgs::readPointCloud2BufferValue<float>(&msg->data[j * step + i_offset], msg->fields[i_idx].datatype);
      ts = sensor_msgs::readPointCloud2BufferValue<double>(&msg->data[j * step + ts_offset], msg->fields[ts_idx].datatype);

      float r = x * x + y * y + z * z;
      if (r < 0.1) continue;
        
      PointXYZIT p{x, y, z, intensity, tbase + ts * 1e-9};
      scan->points.push_back(p);
    }

    std::sort(scan->points.begin(), scan->points.end(), [&](const PointXYZIT &p1, const PointXYZIT &p2) {
      return p1.ts < p2.ts;
    });
    scan->size = scan->points.size();
    
    std::cout << "Received message: " << ++n << std::endl;
    if (laser_queue) laser_queue->push(scan);
  }

  void end_signal_callback(const std_msgs::msg::Bool::SharedPtr msg) {
      if (msg->data) {
          RCLCPP_INFO(this->get_logger(), "End of message stream received.");
          if (laser_queue) laser_queue->push(nullptr);
      }
  }

  static inline int getPointCloud2FieldIndex (const sensor_msgs::msg::PointCloud2 &cloud, const std::string &field_name) {
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size(); ++d)
      if (cloud.fields[d].name == field_name)
        return (d);
    return (-1);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  int n; // message counter
};

}  // namespace traj

#endif  // TRAJLO_ROSBAG_OUSTER_H
