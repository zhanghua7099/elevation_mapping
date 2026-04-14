/*
 *  InputSourceManager.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/ElevationMapping.hpp"

#include <set>

namespace elevation_mapping {

InputSourceManager::InputSourceManager(rclcpp::Node* node) : node_(node) {}

bool InputSourceManager::configureFromRos(const std::string& inputSourcesNamespace) {
  // In ROS2, input sources are configured via parameters declared as a YAML list.
  // We look for parameters with the namespace prefix to find unique source names.
  auto paramResult = node_->list_parameters({inputSourcesNamespace}, 10);

  std::set<std::string> sourceNames;
  for (const auto& name : paramResult.names) {
    if (name.size() <= inputSourcesNamespace.size() + 1) {
      continue;
    }
    std::string remainder = name.substr(inputSourcesNamespace.size() + 1);
    size_t dotPos = remainder.find('.');
    if (dotPos != std::string::npos) {
      sourceNames.insert(remainder.substr(0, dotPos));
    }
  }

  if (sourceNames.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "Could not load the input sources configuration from parameter '%s'. "
                "Assuming that you meant to leave it empty. Not subscribing to any inputs!",
                inputSourcesNamespace.c_str());
    return false;
  }

  std::string robotBaseFrameId;
  if (!node_->has_parameter("robot_base_frame_id")) {
    node_->declare_parameter("robot_base_frame_id", std::string("/robot"));
  }
  robotBaseFrameId = node_->get_parameter("robot_base_frame_id").as_string();

  std::string mapFrameId;
  if (!node_->has_parameter("map_frame_id")) {
    node_->declare_parameter("map_frame_id", std::string("/map"));
  }
  mapFrameId = node_->get_parameter("map_frame_id").as_string();

  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{robotBaseFrameId, mapFrameId};

  // Build a shared TF buffer by querying the node
  // The TF buffer is shared from ElevationMapping node; we re-use it via a separate TransformListener.
  // For InputSourceManager we create a local tf buffer per source (simpler).
  auto tfBuffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  bool successfulConfiguration = true;
  std::set<std::string> subscribedTopics;

  for (const auto& sourceName : sourceNames) {
    const std::string prefix = inputSourcesNamespace + "." + sourceName + ".";

    auto getStringParam = [&](const std::string& key, const std::string& defaultVal) -> std::string {
      const std::string fullKey = prefix + key;
      if (!node_->has_parameter(fullKey)) {
        node_->declare_parameter(fullKey, defaultVal);
      }
      return node_->get_parameter(fullKey).as_string();
    };
    auto getIntParam = [&](const std::string& key, int defaultVal) -> int {
      const std::string fullKey = prefix + key;
      if (!node_->has_parameter(fullKey)) {
        node_->declare_parameter(fullKey, defaultVal);
      }
      return node_->get_parameter(fullKey).as_int();
    };
    auto getBoolParam = [&](const std::string& key, bool defaultVal) -> bool {
      const std::string fullKey = prefix + key;
      if (!node_->has_parameter(fullKey)) {
        node_->declare_parameter(fullKey, defaultVal);
      }
      return node_->get_parameter(fullKey).as_bool();
    };

    const bool isEnabled = getBoolParam("enabled", true);
    if (!isEnabled) {
      continue;
    }

    const std::string type = getStringParam("type", "");
    const std::string topic = getStringParam("topic", "");
    const int queueSize = getIntParam("queue_size", 1);
    const bool publishOnUpdate = getBoolParam("publish_on_update", true);
    const std::string sensorProcessorType = getStringParam("sensor_processor.type", "perfect");

    Input source{node_};
    const bool configured{source.configure(sourceName, type, topic, queueSize, publishOnUpdate, sensorProcessorType,
                                           generalSensorProcessorConfig, tfBuffer)};
    if (!configured) {
      successfulConfiguration = false;
      continue;
    }

    const std::string subscribedTopic{source.getSubscribedTopic()};
    const bool topicIsUnique{subscribedTopics.insert(subscribedTopic).second};

    if (topicIsUnique) {
      sources_.push_back(std::move(source));
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "The input sources specification tried to subscribe to %s multiple times. Only subscribing once.",
                  subscribedTopic.c_str());
      successfulConfiguration = false;
    }
  }

  return successfulConfiguration;
}

int InputSourceManager::getNumberOfSources() {
  return static_cast<int>(sources_.size());
}

}  // namespace elevation_mapping
