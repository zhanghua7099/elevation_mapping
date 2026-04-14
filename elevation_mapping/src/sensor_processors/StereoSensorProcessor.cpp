/*
 * StereoSensorProcessor.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Hannes Keller
 */

#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

StereoSensorProcessor::StereoSensorProcessor(rclcpp::Node* node, const SensorProcessorBase::GeneralParameters& generalParameters,
                                             std::shared_ptr<tf2_ros::Buffer> tfBuffer)
    : SensorProcessorBase(node, generalParameters, tfBuffer), originalWidth_(1) {}

StereoSensorProcessor::~StereoSensorProcessor() = default;

bool StereoSensorProcessor::readParameters() {
  SensorProcessorBase::readParameters();
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  auto declareAndGet = [&](const std::string& name, double defaultVal) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, defaultVal);
    }
    return node_->get_parameter(name).as_double();
  };
  parameters.sensorParameters_["p_1"] = declareAndGet("sensor_processor/p_1", 0.0);
  parameters.sensorParameters_["p_2"] = declareAndGet("sensor_processor/p_2", 0.0);
  parameters.sensorParameters_["p_3"] = declareAndGet("sensor_processor/p_3", 0.0);
  parameters.sensorParameters_["p_4"] = declareAndGet("sensor_processor/p_4", 0.0);
  parameters.sensorParameters_["p_5"] = declareAndGet("sensor_processor/p_5", 0.0);
  parameters.sensorParameters_["lateral_factor"] = declareAndGet("sensor_processor/lateral_factor", 0.0);
  parameters.sensorParameters_["depth_to_disparity_factor"] = declareAndGet("sensor_processor/depth_to_disparity_factor", 0.0);
  parameters.sensorParameters_["cutoff_min_depth"] =
      declareAndGet("sensor_processor/cutoff_min_depth", std::numeric_limits<double>::min());
  parameters.sensorParameters_["cutoff_max_depth"] =
      declareAndGet("sensor_processor/cutoff_max_depth", std::numeric_limits<double>::max());
  return true;
}

bool StereoSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
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

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    pcl::PointXYZRGBConfidenceRatio point = pointCloud->points[i];
    double disparity = parameters.sensorParameters_.at("depth_to_disparity_factor") / point.z;
    Eigen::Vector3f pointVector(point.x, point.y, point.z);
    float heightVariance = 0.0;

    float measurementDistance = pointVector.norm();

    float varianceNormal =
        pow(parameters.sensorParameters_.at("depth_to_disparity_factor") / pow(disparity, 2), 2) *
        ((parameters.sensorParameters_.at("p_5") * disparity + parameters.sensorParameters_.at("p_2")) *
             sqrt(pow(parameters.sensorParameters_.at("p_3") * disparity + parameters.sensorParameters_.at("p_4") - getJ(i), 2) +
                  pow(240 - getI(i), 2)) +
         parameters.sensorParameters_.at("p_1"));
    float varianceLateral = pow(parameters.sensorParameters_.at("lateral_factor") * measurementDistance, 2);
    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    variances(i) = heightVariance;
  }

  return true;
}

bool StereoSensorProcessor::filterPointCloudSensorType(const PointCloudType::Ptr pointCloud) {
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

int StereoSensorProcessor::getI(int index) {
  return indices_[index] / originalWidth_;
}

int StereoSensorProcessor::getJ(int index) {
  return indices_[index] % originalWidth_;
}

}  // namespace elevation_mapping
