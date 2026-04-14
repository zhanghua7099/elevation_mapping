/*
 * RobotMotionMapUpdater.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 * Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/Core>

// ROS
#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {

/*!
 * Computes the map variance update from the pose covariance of the robot.
 */
class RobotMotionMapUpdater {
 public:
  using Pose = kindr::HomogeneousTransformationPosition3RotationQuaternionD;
  using Covariance = Eigen::Matrix<double, 3, 3>;
  using PoseCovariance = Eigen::Matrix<double, 6, 6>;
  using ReducedCovariance = Eigen::Matrix<double, 4, 4>;
  using Jacobian = Eigen::Matrix<double, 4, 4>;

  explicit RobotMotionMapUpdater(rclcpp::Node* node);

  virtual ~RobotMotionMapUpdater();

  bool readParameters();

  bool update(ElevationMap& map, const Pose& robotPose, const PoseCovariance& robotPoseCovariance, const rclcpp::Time& time);

 private:
  static bool computeReducedCovariance(const Pose& robotPose, const PoseCovariance& robotPoseCovariance,
                                       ReducedCovariance& reducedCovariance);

  bool computeRelativeCovariance(const Pose& robotPose, const ReducedCovariance& reducedCovariance, ReducedCovariance& relativeCovariance);

  //! ROS node pointer.
  rclcpp::Node* node_;

  //! Time of the previous update.
  rclcpp::Time previousUpdateTime_;

  //! Previous robot pose.
  Pose previousRobotPose_;

  //! Robot pose covariance (reduced) from the previous update.
  ReducedCovariance previousReducedCovariance_;

  //! Scaling factor for the covariance matrix (default 1).
  double covarianceScale_;
};

}  // namespace elevation_mapping
