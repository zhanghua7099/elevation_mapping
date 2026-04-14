/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(rclcpp::Node* node, const GeneralParameters& generalConfig,
                                         std::shared_ptr<tf2_ros::Buffer> tfBuffer)
    : node_(node), tfBuffer_(tfBuffer), firstTfAvailable_(false) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  transformationSensorToMap_.setIdentity();
  generalParameters_ = generalConfig;
  RCLCPP_DEBUG(node_->get_logger(),
               "Sensor processor general parameters are:"
               "\n\t- robot_base_frame_id: %s"
               "\n\t- map_frame_id: %s",
               generalConfig.robotBaseFrameId_.c_str(), generalConfig.mapFrameId_.c_str());
}

SensorProcessorBase::~SensorProcessorBase() = default;

bool SensorProcessorBase::readParameters() {
  Parameters parameters;
  auto declareAndGet = [&](const std::string& name, auto defaultVal) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, defaultVal);
    }
    using T = decltype(defaultVal);
    return node_->get_parameter(name).get_value<T>();
  };

  parameters.ignorePointsUpperThreshold_ =
      declareAndGet("sensor_processor/ignore_points_above", std::numeric_limits<double>::infinity());
  parameters.ignorePointsLowerThreshold_ =
      declareAndGet("sensor_processor/ignore_points_below", -std::numeric_limits<double>::infinity());
  parameters.applyVoxelGridFilter_ = declareAndGet("sensor_processor/apply_voxelgrid_filter", false);
  parameters.sensorParameters_["voxelgrid_filter_size"] = declareAndGet("sensor_processor/voxelgrid_filter_size", 0.0);
  parameters_.setData(parameters);
  return true;
}

bool SensorProcessorBase::process(const PointCloudType::ConstPtr pointCloudInput,
                                  const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                  const PointCloudType::Ptr pointCloudMapFrame, Eigen::VectorXf& variances, std::string sensorFrame) {
  const Parameters parameters{parameters_.getData()};
  sensorFrameId_ = sensorFrame;
  RCLCPP_DEBUG(node_->get_logger(), "Sensor Processor processing for frame %s", sensorFrameId_.c_str());

  rclcpp::Time timeStamp(static_cast<int64_t>(pointCloudInput->header.stamp) * 1000LL, RCL_ROS_TIME);
  if (!updateTransformations(timeStamp)) {
    return false;
  }

  PointCloudType::Ptr pointCloudSensorFrame(new PointCloudType);
  transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);

  filterPointCloud(pointCloudSensorFrame);
  filterPointCloudSensorType(pointCloudSensorFrame);

  if (!transformPointCloud(pointCloudSensorFrame, pointCloudMapFrame, generalParameters_.mapFrameId_)) {
    return false;
  }
  std::vector<PointCloudType::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);

  return computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances);
}

bool SensorProcessorBase::updateTransformations(const rclcpp::Time& timeStamp) {
  try {
    if (!tfBuffer_->canTransform(generalParameters_.mapFrameId_, sensorFrameId_, timeStamp, tf2::durationFromSec(1.0))) {
      return false;
    }

    auto transformMsg = tfBuffer_->lookupTransform(generalParameters_.mapFrameId_, sensorFrameId_, timeStamp);
    transformationSensorToMap_ = tf2::transformToEigen(transformMsg);

    auto transformBaseToSensor = tfBuffer_->lookupTransform(generalParameters_.robotBaseFrameId_, sensorFrameId_, timeStamp);
    Eigen::Affine3d transform = tf2::transformToEigen(transformBaseToSensor);
    rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
    translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

    auto transformMapToBase = tfBuffer_->lookupTransform(generalParameters_.mapFrameId_, generalParameters_.robotBaseFrameId_, timeStamp);
    transform = tf2::transformToEigen(transformMapToBase);
    rotationMapToBase_.setMatrix(transform.rotation().matrix());
    translationMapToBaseInMapFrame_.toImplementation() = transform.translation();

    firstTfAvailable_ = true;
    return true;
  } catch (const tf2::TransformException& ex) {
    if (!firstTfAvailable_) {
      return false;
    }
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return false;
  }
}

bool SensorProcessorBase::transformPointCloud(PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr pointCloudTransformed,
                                              const std::string& targetFrame) {
  rclcpp::Time timeStamp(static_cast<int64_t>(pointCloud->header.stamp) * 1000LL, RCL_ROS_TIME);
  const std::string inputFrameId(pointCloud->header.frame_id);

  try {
    auto transformMsg = tfBuffer_->lookupTransform(targetFrame, inputFrameId, timeStamp, tf2::durationFromSec(1.0));
    Eigen::Affine3d transform = tf2::transformToEigen(transformMsg);
    pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
    pointCloudTransformed->header.frame_id = targetFrame;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "%s", ex.what());
    return false;
  }

  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "Point cloud transformed to frame %s for time stamp %f.",
                        targetFrame.c_str(), pointCloudTransformed->header.stamp / 1000.0);
  return true;
}

void SensorProcessorBase::removePointsOutsideLimits(PointCloudType::ConstPtr reference,
                                                    std::vector<PointCloudType::Ptr>& pointClouds) {
  const Parameters parameters{parameters_.getData()};
  if (!std::isfinite(parameters.ignorePointsLowerThreshold_) && !std::isfinite(parameters.ignorePointsUpperThreshold_)) {
    return;
  }
  RCLCPP_DEBUG(node_->get_logger(), "Limiting point cloud to the height interval of [%f, %f] relative to the robot base.",
               parameters.ignorePointsLowerThreshold_, parameters.ignorePointsUpperThreshold_);

  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z");
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + parameters.ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + parameters.ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndeces(new std::vector<int>);
  passThroughFilter.filter(*insideIndeces);

  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndeces);
    PointCloudType tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }

  RCLCPP_DEBUG(node_->get_logger(), "removePointsOutsideLimits() reduced point cloud to %i points.", (int)pointClouds[0]->size());
}

bool SensorProcessorBase::filterPointCloud(const PointCloudType::Ptr pointCloud) {
  const Parameters parameters{parameters_.getData()};
  PointCloudType tempPointCloud;

  std::vector<int> indices;
  if (!pointCloud->is_dense) {
    pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices);
    tempPointCloud.is_dense = true;
    pointCloud->swap(tempPointCloud);
  }

  if (parameters.applyVoxelGridFilter_) {
    pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio> voxelGridFilter;
    voxelGridFilter.setInputCloud(pointCloud);
    double filter_size = parameters.sensorParameters_.at("voxelgrid_filter_size");
    voxelGridFilter.setLeafSize(filter_size, filter_size, filter_size);
    voxelGridFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }
  RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "cleanPointCloud() reduced point cloud to %i points.",
                        static_cast<int>(pointCloud->size()));
  return true;
}

bool SensorProcessorBase::filterPointCloudSensorType(const PointCloudType::Ptr /*pointCloud*/) {
  return true;
}

} /* namespace elevation_mapping */
