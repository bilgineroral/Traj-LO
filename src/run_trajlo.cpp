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
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

int main(int argc, char *argv[]) {
  std::cout << "Hello Traj-LO Project!\n";
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
    return EXIT_FAILURE;
  }

  std::string config_path = argv[1];

  traj::TrajConfig config;
  config.load(config_path);

  // create node which runs TrajLO algorithm and publishes to /trajlo_pose topic
  auto trajLOdometry = std::make_shared<traj::TrajLOdometry>(config);

  auto data_loader = traj::DatasetFactory::GetDatasetIo(config.type);
  if (!data_loader) {
    std::cerr << "Error: Failed to get dataset IO of type " << config.type << std::endl;
    return EXIT_FAILURE;
  }
  
  // init data loader
  data_loader->publish("", config.topic);
  std::cout << "Data loader is initialized..." << std::endl;

  // data loader sends the scan data to the algorithm
  std::cout << "Binding odometry->laser_queue to data_loader->laser_queue...\n";
  data_loader->laser_queue = &trajLOdometry->laser_data_queue;

  std::cout << "Starting data loader thread..." << std::endl;
  std::thread t_data_loader([&]() {
    rclcpp::spin(data_loader);
  });

  rclcpp::spin(trajLOdometry);

  t_data_loader.join();
  
  rclcpp::shutdown();

  return 0;
}