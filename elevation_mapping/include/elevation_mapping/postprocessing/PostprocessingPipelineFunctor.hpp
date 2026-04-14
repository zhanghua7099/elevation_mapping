/*
 * PostprocessingPipelineFunctor.hpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <filters/filter_chain.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

// Elevation Mapping
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

namespace elevation_mapping {

class PostprocessingPipelineFunctor {
 public:
  using GridMap = grid_map::GridMap;

  explicit PostprocessingPipelineFunctor(rclcpp::Node* node);

  ~PostprocessingPipelineFunctor();

  GridMap operator()(GridMap& inputMap);

  void publish(const GridMap& gridMap) const;

  bool hasSubscribers() const;

 private:
  void readParameters();

  //! ROS node pointer.
  rclcpp::Node* node_;

  //! Grid map publisher.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  struct Parameters {
    std::string outputTopic_;
    std::string filterChainParametersName_;
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  bool filterChainConfigured_;
};

}  // namespace elevation_mapping
