/*
 * LaserSensorProcessor.cpp
 *
 *  Created on: Sep 15, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"

#include <pcl/filters/passthrough.h>
#include <limits>
#include <string>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

LaserSensorProcessor::LaserSensorProcessor(rclcpp::Node* node, const SensorProcessorBase::GeneralParameters& generalParameters,
                                           std::shared_ptr<tf2_ros::Buffer> tfBuffer)
    : SensorProcessorBase(node, generalParameters, tfBuffer) {}

LaserSensorProcessor::~LaserSensorProcessor() = default;

bool LaserSensorProcessor::readParameters() {
  SensorProcessorBase::readParameters();
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  auto declareAndGet = [&](const std::string& name, double defaultVal) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, defaultVal);
    }
    return node_->get_parameter(name).as_double();
  };
  parameters.sensorParameters_["min_radius"] = declareAndGet("sensor_processor/min_radius", 0.0);
  parameters.sensorParameters_["beam_angle"] = declareAndGet("sensor_processor/beam_angle", 0.0);
  parameters.sensorParameters_["beam_constant"] = declareAndGet("sensor_processor/beam_constant", 0.0);
  return true;
}

bool LaserSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
                                            const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, Eigen::VectorXf& variances) {
  const Parameters parameters{parameters_.getData()};
  variances.resize(pointCloud->size());

  const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

  const Eigen::RowVector3f sensorJacobian =
      projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

  Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

  const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
  const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
  const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
  const Eigen::Matrix3f B_r_BS_skew =
      kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  const float varianceNormal = parameters.sensorParameters_.at("min_radius") * parameters.sensorParameters_.at("min_radius");
  const float beamConstant = parameters.sensorParameters_.at("beam_constant");
  const float beamAngle = parameters.sensorParameters_.at("beam_angle");
  for (size_t i = 0; i < pointCloud->size(); ++i) {
    const auto& point = pointCloud->points[i];
    const Eigen::Vector3f pointVector(point.x, point.y, point.z);

    const float measurementDistance = pointVector.norm();

    float varianceLateral = beamConstant + beamAngle * measurementDistance;
    varianceLateral *= varianceLateral;

    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    const Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    float heightVariance = 0.0;
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    variances(i) = heightVariance;
  }

  return true;
}

}  // namespace elevation_mapping
