/*
 *  InputSourceManager.hpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "elevation_mapping/input_sources/Input.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

/**
 * @brief An input source manager reads a list of input sources from the configuration and connects them to the
 * appropriate callback of elevation mapping.
 */
class InputSourceManager {
 public:
  /**
   * @brief Constructor.
   * @param node Used to resolve the namespace and setup the subscribers.
   */
  explicit InputSourceManager(rclcpp::Node* node);

  /**
   * @brief Configure the input sources from ROS2 parameters under inputSourcesNamespace.
   */
  bool configureFromRos(const std::string& inputSourcesNamespace);

  /**
   * @brief Registers the corresponding callback in the elevationMap.
   */
  template <typename... MsgT>
  bool registerCallbacks(ElevationMapping& map, std::pair<const char*, Input::CallbackT<MsgT>>... callbacks);

  int getNumberOfSources();

 protected:
  std::vector<Input> sources_;
  rclcpp::Node* node_;
};

// Template definitions

template <typename... MsgT>
bool InputSourceManager::registerCallbacks(ElevationMapping& map, std::pair<const char*, Input::CallbackT<MsgT>>... callbacks) {
  if (sources_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Not registering any callbacks, no input sources given. Did you configure the InputSourceManager?");
    return true;
  }
  for (Input& source : sources_) {
    bool callbackRegistered = false;
    for (auto& callback : {callbacks...}) {
      if (source.getType() == callback.first) {
        source.registerCallback(map, callback.second);
        callbackRegistered = true;
      }
    }
    if (!callbackRegistered) {
      RCLCPP_WARN(node_->get_logger(), "The configuration contains input sources of an unknown type: %s", source.getType().c_str());
      RCLCPP_WARN(node_->get_logger(), "Available types are:");
      for (auto& callback : {callbacks...}) {
        RCLCPP_WARN(node_->get_logger(), "- %s", callback.first);
      }
      return false;
    }
  }
  return true;
}

}  // namespace elevation_mapping
