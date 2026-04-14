/*
 *  Input.hpp
 *
 *  Created on: Oct 06, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {
class ElevationMapping;  // Forward declare to avoid cyclic import dependency.

/**
 * @brief An Input feeds data to ElevationMapping callbacks.
 */
class Input {
 public:
  template <typename MsgT>
  using CallbackT = void (ElevationMapping::*)(const std::shared_ptr<const MsgT>&, bool, const SensorProcessorBase::Ptr&);

  /**
   * @brief Constructor.
   * @param node Pointer to the ROS2 node. Used to subscribe to inputs.
   */
  explicit Input(rclcpp::Node* node);

  bool isEnabled() const {
    const Parameters parameters{parameters_.getData()};
    return parameters.isEnabled_;
  }

  /**
   * @brief Configure the input source.
   */
  bool configure(const std::string& name, const std::string& type, const std::string& topic, int queueSize, bool publishOnUpdate,
                 const std::string& sensorProcessorType,
                 const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters,
                 std::shared_ptr<tf2_ros::Buffer> tfBuffer);

  template <typename MsgT>
  void registerCallback(ElevationMapping& map, CallbackT<MsgT> callback);

  std::string getSubscribedTopic() const;

  std::string getType() {
    const Parameters parameters{parameters_.getData()};
    return parameters.type_;
  }

 private:
  bool configureSensorProcessor(const std::string& name, const std::string& sensorType,
                                const SensorProcessorBase::GeneralParameters& generalSensorProcessorParameters,
                                std::shared_ptr<tf2_ros::Buffer> tfBuffer);

  // ROS connection.
  rclcpp::SubscriptionBase::SharedPtr subscriber_;
  rclcpp::Node* node_;

  //! Sensor processor
  SensorProcessorBase::Ptr sensorProcessor_;

  // Parameters.
  struct Parameters {
    std::string name_;
    std::string type_;
    bool isEnabled_{true};
    uint32_t queueSize_{0};
    std::string topic_;
    bool publishOnUpdate_{true};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;
};

template <typename MsgT>
void Input::registerCallback(ElevationMapping& map, CallbackT<MsgT> callback) {
  const Parameters parameters{parameters_.getData()};
  subscriber_ = node_->create_subscription<MsgT>(
      parameters.topic_, parameters.queueSize_,
      std::bind(callback, std::ref(map), std::placeholders::_1, parameters.publishOnUpdate_, std::ref(sensorProcessor_)));
  RCLCPP_INFO(node_->get_logger(), "Subscribing to %s: %s, queue_size: %i.", parameters.type_.c_str(), parameters.topic_.c_str(),
              parameters.queueSize_);
}

}  // namespace elevation_mapping
