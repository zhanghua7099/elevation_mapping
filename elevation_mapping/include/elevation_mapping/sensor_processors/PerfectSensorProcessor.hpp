/*
 * PerfectSensorProcessor.hpp
 *
 *  Created on: Sep 28, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

class PerfectSensorProcessor : public SensorProcessorBase {
 public:
  PerfectSensorProcessor(rclcpp::Node* node, const SensorProcessorBase::GeneralParameters& generalParameters,
                         std::shared_ptr<tf2_ros::Buffer> tfBuffer);
  ~PerfectSensorProcessor() override;

 private:
  bool readParameters() override;
  bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;
};

} /* namespace elevation_mapping */
