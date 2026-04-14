/*
 * StructuredLightSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <limits>
#include <string>
#include <vector>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

StructuredLightSensorProcessor::StructuredLightSensorProcessor(rclcpp::Node* node,
                                                               const SensorProcessorBase::GeneralParameters& generalParameters,
                                                               std::shared_ptr<tf2_ros::Buffer> tfBuffer)
    : SensorProcessorBase(node, generalParameters, tfBuffer) {}

StructuredLightSensorProcessor::~StructuredLightSensorProcessor() = default;

bool StructuredLightSensorProcessor::readParameters() {
  SensorProcessorBase::readParameters();
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  auto declareAndGet = [&](const std::string& name, double defaultVal) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, defaultVal);
    }
    return node_->get_parameter(name).as_double();
  };
  parameters.sensorParameters_["normal_factor_a"] = declareAndGet("sensor_processor/normal_factor_a", 0.0);
  parameters.sensorParameters_["normal_factor_b"] = declareAndGet("sensor_processor/normal_factor_b", 0.0);
  parameters.sensorParameters_["normal_factor_c"] = declareAndGet("sensor_processor/normal_factor_c", 0.0);
  parameters.sensorParameters_["normal_factor_d"] = declareAndGet("sensor_processor/normal_factor_d", 0.0);
  parameters.sensorParameters_["normal_factor_e"] = declareAndGet("sensor_processor/normal_factor_e", 0.0);
  parameters.sensorParameters_["lateral_factor"] = declareAndGet("sensor_processor/lateral_factor", 0.0);
  parameters.sensorParameters_["cutoff_min_depth"] =
      declareAndGet("sensor_processor/cutoff_min_depth", std::numeric_limits<double>::min());
  parameters.sensorParameters_["cutoff_max_depth"] =
      declareAndGet("sensor_processor/cutoff_max_depth", std::numeric_limits<double>::max());
  return true;
}

bool StructuredLightSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
                                                      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                                      Eigen::VectorXf& variances) {
  const Parameters parameters{parameters_.getData()};
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
  const float epsilon = std::numeric_limits<float>::min();

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    const auto& point{pointCloud->points[i]};
    const float& confidenceRatio = point.confidence_ratio;
    Eigen::Vector3f pointVector(point.x, point.y, point.z);

    const float measurementDistance = pointVector.z();

    const float deviationNormal =
        parameters.sensorParameters_.at("normal_factor_a") +
        parameters.sensorParameters_.at("normal_factor_b") * (measurementDistance - parameters.sensorParameters_.at("normal_factor_c")) *
            (measurementDistance - parameters.sensorParameters_.at("normal_factor_c")) +
        parameters.sensorParameters_.at("normal_factor_d") * pow(measurementDistance, parameters.sensorParameters_.at("normal_factor_e"));
    const float varianceNormal = deviationNormal * deviationNormal;
    const float deviationLateral = parameters.sensorParameters_.at("lateral_factor") * measurementDistance;
    const float varianceLateral = deviationLateral * deviationLateral;
    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    const Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    float heightVariance = 0.0;
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance +=
        static_cast<float>(sensorJacobian * sensorVariance * sensorJacobian.transpose()) / (epsilon + confidenceRatio * confidenceRatio);

    variances(i) = heightVariance;
  }

  return true;
}

bool StructuredLightSensorProcessor::filterPointCloudSensorType(const PointCloudType::Ptr pointCloud) {
  const Parameters parameters{parameters_.getData()};
  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter;
  PointCloudType tempPointCloud;

  passThroughFilter.setInputCloud(pointCloud);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(parameters.sensorParameters_.at("cutoff_min_depth"),
                                    parameters.sensorParameters_.at("cutoff_max_depth"));
  passThroughFilter.filter(tempPointCloud);
  pointCloud->swap(tempPointCloud);

  return true;
}

}  // namespace elevation_mapping
