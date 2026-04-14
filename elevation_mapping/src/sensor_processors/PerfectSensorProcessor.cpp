/*
 * PerfectSensorProcessor.cpp
 *
 *  Created on: Sep 28, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"

#include <pcl/filters/filter.h>
#include <limits>
#include <string>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

PerfectSensorProcessor::PerfectSensorProcessor(rclcpp::Node* node, const SensorProcessorBase::GeneralParameters& generalParameters,
                                               std::shared_ptr<tf2_ros::Buffer> tfBuffer)
    : SensorProcessorBase(node, generalParameters, tfBuffer) {}

PerfectSensorProcessor::~PerfectSensorProcessor() = default;

bool PerfectSensorProcessor::readParameters() {
  return SensorProcessorBase::readParameters();
}

bool PerfectSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
                                              const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, Eigen::VectorXf& variances) {
  variances.resize(pointCloud->size());

  const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

  const Eigen::RowVector3f sensorJacobian =
      projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

  const Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

  const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
  const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
  const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
  const Eigen::Matrix3f B_r_BS_skew =
      kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    const auto& point{pointCloud->points[i]};
    Eigen::Vector3f pointVector(point.x, point.y, point.z);
    float heightVariance = 0.0;

    float varianceNormal = 0.0;
    float varianceLateral = 0.0;
    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    const Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    variances(i) = heightVariance;
  }

  return true;
}

}  // namespace elevation_mapping
