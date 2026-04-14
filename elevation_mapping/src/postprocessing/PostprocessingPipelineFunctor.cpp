/*
 * PostprocessingPipelineFunctor.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include <grid_map_ros/grid_map_ros.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(rclcpp::Node* node)
    : node_(node), filterChain_("grid_map::GridMap"), filterChainConfigured_(false) {
  readParameters();
  const Parameters parameters{parameters_.getData()};
  publisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(parameters.outputTopic_, 1);

  if (!node_->has_parameter(parameters.filterChainParametersName_) ||
      !filterChain_.configure(parameters.filterChainParametersName_, node_->get_node_logging_interface(),
                              node_->get_node_parameters_interface())) {
    RCLCPP_WARN(node_->get_logger(), "Could not configure the filter chain. Will publish the raw elevation map without postprocessing!");
    return;
  }

  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  Parameters parameters;
  if (!node_->has_parameter("output_topic")) {
    node_->declare_parameter("output_topic", std::string("elevation_map_raw"));
  }
  if (!node_->has_parameter("postprocessor_pipeline_name")) {
    node_->declare_parameter("postprocessor_pipeline_name", std::string("postprocessor_pipeline"));
  }
  parameters.outputTopic_ = node_->get_parameter("output_topic").as_string();
  parameters.filterChainParametersName_ = node_->get_parameter("postprocessor_pipeline_name").as_string();
  parameters_.setData(parameters);
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  if (!filterChainConfigured_) {
    RCLCPP_WARN_ONCE(node_->get_logger(), "No postprocessing pipeline was configured. Forwarding the raw elevation map!");
    return inputMap;
  }

  grid_map::GridMap outputMap;
  if (!filterChain_.update(inputMap, outputMap)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not perform the grid map filter chain! Forwarding the raw elevation map!");
    return inputMap;
  }

  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  grid_map_msgs::msg::GridMap outputMessage;
  grid_map::GridMapRosConverter::toMessage(gridMap, outputMessage);
  publisher_->publish(outputMessage);
  RCLCPP_DEBUG(node_->get_logger(), "Elevation map raw has been published.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_->get_subscription_count() > 0;
}

}  // namespace elevation_mapping
