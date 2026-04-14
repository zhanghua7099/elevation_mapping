/*
 * StructuredLightSensorProcessor.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

class StructuredLightSensorProcessor : public SensorProcessorBase {
 public:
  StructuredLightSensorProcessor(rclcpp::Node* node, const SensorProcessorBase::GeneralParameters& generalParameters,
                                  std::shared_ptr<tf2_ros::Buffer> tfBuffer);
  ~StructuredLightSensorProcessor() override;

 private:
  bool readParameters() override;
  bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;
  bool filterPointCloudSensorType(PointCloudType::Ptr pointCloud) override;
};

} /* namespace elevation_mapping */
