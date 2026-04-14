/*
 * StereoSensorProcessor.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Hannes Keller
 */

#pragma once

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

class StereoSensorProcessor : public SensorProcessorBase {
 public:
  StereoSensorProcessor(rclcpp::Node* node, const SensorProcessorBase::GeneralParameters& generalParameters,
                        std::shared_ptr<tf2_ros::Buffer> tfBuffer);
  ~StereoSensorProcessor() override;

 private:
  bool readParameters() override;
  bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                        Eigen::VectorXf& variances) override;
  bool filterPointCloudSensorType(PointCloudType::Ptr pointCloud) override;

  int getI(int index);
  int getJ(int index);

  std::vector<int> indices_;
  int originalWidth_;
};

}  // namespace elevation_mapping
