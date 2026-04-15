/*
 *  Input.cpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include <memory>

#include "elevation_mapping/input_sources/Input.hpp"

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

Input::Input(rclcpp::Node* node) : node_(node) {}

bool Input::configure(const std::string& name, const std::string& type, const std::string& topic, int queueSize, bool publishOnUpdate,
                      const std::string& sensorProcessorType,
                      const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters,
                      std::shared_ptr<tf2_ros::Buffer> tfBuffer) {
  Parameters parameters;
  parameters.name_ = name;
  parameters.type_ = type;
  parameters.topic_ = topic;
  if (queueSize >= 0) {
    parameters.queueSize_ = static_cast<unsigned int>(queueSize);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "The specified queue_size is negative.");
    return false;
  }
  parameters.publishOnUpdate_ = publishOnUpdate;
  parameters_.setData(parameters);

  if (!configureSensorProcessor(name, sensorProcessorType, generalSensorProcessorParameters, tfBuffer)) {
    return false;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Configured %s:%s @ %s (publishing_on_update: %s), using %s to process data.", type.c_str(),
               name.c_str(), topic.c_str(), publishOnUpdate ? "true" : "false", sensorProcessorType.c_str());
  return true;
}

std::string Input::getSubscribedTopic() const {
  const Parameters parameters{parameters_.getData()};
  return parameters.topic_;
}

bool Input::configureSensorProcessor(const std::string& name, const std::string& sensorType,
                                     const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters,
                                     std::shared_ptr<tf2_ros::Buffer> tfBuffer) {
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(node_, generalSensorProcessorParameters, tfBuffer);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(node_, generalSensorProcessorParameters, tfBuffer);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(node_, generalSensorProcessorParameters, tfBuffer);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(node_, generalSensorProcessorParameters, tfBuffer);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "The sensor type %s for input '%s' is not available.", sensorType.c_str(), name.c_str());
    return false;
  }

  return sensorProcessor_->readParameters();
}

}  // namespace elevation_mapping
