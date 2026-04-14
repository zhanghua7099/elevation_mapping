/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <rclcpp/rclcpp.hpp>
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<elevation_mapping::ElevationMapping>();

  int numThreads = node->declare_parameter("num_callback_threads", 1);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), numThreads);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
