/*
 * SensorProcessorBase.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/Core>

// STL
#include <memory>
#include <string>
#include <unordered_map>

// Elevation Mapping
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"

namespace elevation_mapping {

/*!
 * Generic Sensor processor base class.
 */
class SensorProcessorBase {
 public:
  using Ptr = std::unique_ptr<SensorProcessorBase>;
  friend class ElevationMapping;
  friend class Input;

  struct GeneralParameters {
    std::string robotBaseFrameId_;
    std::string mapFrameId_;

    explicit GeneralParameters(std::string robotBaseFrameId = "robot", std::string mapFrameId = "map")
        : robotBaseFrameId_(std::move(robotBaseFrameId)), mapFrameId_(std::move(mapFrameId)) {}
  };

  SensorProcessorBase(rclcpp::Node* node, const GeneralParameters& generalConfig,
                      std::shared_ptr<tf2_ros::Buffer> tfBuffer);

  virtual ~SensorProcessorBase();

  bool process(PointCloudType::ConstPtr pointCloudInput, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
               PointCloudType::Ptr pointCloudMapFrame, Eigen::VectorXf& variances, std::string sensorFrame);

  bool isTfAvailableInBuffer() const { return firstTfAvailable_; }

 protected:
  virtual bool readParameters();

  bool filterPointCloud(PointCloudType::Ptr pointCloud);

  virtual bool filterPointCloudSensorType(PointCloudType::Ptr pointCloud);

  virtual bool computeVariances(PointCloudType::ConstPtr pointCloud, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                Eigen::VectorXf& variances) = 0;

  bool updateTransformations(const rclcpp::Time& timeStamp);

  bool transformPointCloud(PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr pointCloudTransformed, const std::string& targetFrame);

  void removePointsOutsideLimits(PointCloudType::ConstPtr reference, std::vector<PointCloudType::Ptr>& pointClouds);

  //! ROS node pointer.
  rclcpp::Node* node_;

  //! TF2 buffer (shared with ElevationMapping node).
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

  //! Rotation from Base to Sensor frame (C_SB)
  kindr::RotationMatrixD rotationBaseToSensor_;

  //! Translation from Base to Sensor in Base frame (B_r_BS)
  kindr::Position3D translationBaseToSensorInBaseFrame_;

  //! Rotation from (elevation) Map to Base frame (C_BM)
  kindr::RotationMatrixD rotationMapToBase_;

  //! Translation from Map to Base in Map frame (M_r_MB)
  kindr::Position3D translationMapToBaseInMapFrame_;

  //! Transformation from Sensor to Map frame
  Eigen::Affine3d transformationSensorToMap_;

  GeneralParameters generalParameters_;

  struct Parameters {
    double ignorePointsUpperThreshold_{std::numeric_limits<double>::infinity()};
    double ignorePointsLowerThreshold_{-std::numeric_limits<double>::infinity()};
    bool applyVoxelGridFilter_{false};
    std::unordered_map<std::string, double> sensorParameters_;
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  //! TF frame id of the range sensor for the point clouds.
  std::string sensorFrameId_;

  //! Indicates if the requested tf transformation was available.
  bool firstTfAvailable_;
};

} /* namespace elevation_mapping */
