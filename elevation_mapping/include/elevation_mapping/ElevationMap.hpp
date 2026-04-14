/*
 * ElevationMap.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 * Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Kindr
#include <kindr/Core>

// Boost
#include <boost/thread/recursive_mutex.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>

// Elevation Mapping
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

namespace elevation_mapping {

/*!
 * Elevation map stored as grid map handling elevation height, variance, color etc.
 */
class ElevationMap {
 public:
  /*!
   * Constructor.
   */
  explicit ElevationMap(rclcpp::Node* node);

  /*!
   * Destructor.
   */
  virtual ~ElevationMap();

  void setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position);

  bool add(PointCloudType::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, const rclcpp::Time& timeStamp,
           const Eigen::Affine3d& transformationSensorToMap);

  bool update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
              const grid_map::Matrix& horizontalVarianceUpdateY, const grid_map::Matrix& horizontalVarianceUpdateXY,
              const rclcpp::Time& time);

  bool fuseAll();

  bool fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length);

  bool clear();

  void visibilityCleanup(const rclcpp::Time& updatedTime);

  void move(const Eigen::Vector2d& position);

  bool postprocessAndPublishRawElevationMap();

  bool publishFusedElevationMap();

  bool publishVisibilityCleanupMap();

  grid_map::GridMap& getRawGridMap();

  void setRawGridMap(const grid_map::GridMap& map);

  grid_map::GridMap& getFusedGridMap();

  void setFusedGridMap(const grid_map::GridMap& map);

  rclcpp::Time getTimeOfLastUpdate();

  rclcpp::Time getTimeOfLastFusion();

  const kindr::HomTransformQuatD& getPose();

  bool getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position);

  boost::recursive_mutex& getFusedDataMutex();

  boost::recursive_mutex& getRawDataMutex();

  void setFrameId(const std::string& frameId);

  const std::string& getFrameId();

  void setTimestamp(rclcpp::Time timestamp);

  bool hasRawMapSubscribers() const;

  bool hasFusedMapSubscribers() const;

  void underlyingMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr& underlyingMap);

  void setRawSubmapHeight(const grid_map::Position& initPosition, float mapHeight, float variance, double lengthInXSubmap,
                          double lengthInYSubmap);

  friend class ElevationMapping;

 private:
  bool fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size);

  bool clean();

  void resetFusedData();

  static float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

  //! ROS node pointer.
  rclcpp::Node* node_;

  //! Raw elevation map as grid map.
  grid_map::GridMap rawMap_;

  //! Fused elevation map as grid map.
  grid_map::GridMap fusedMap_;

  //! Visibility cleanup debug map.
  grid_map::GridMap visibilityCleanupMap_;

  //! Underlying map, used for ground truth maps, multi-robot mapping etc.
  grid_map::GridMap underlyingMap_;

  //! Thread Pool to handle raw map postprocessing filter pipelines.
  PostprocessorPool postprocessorPool_;

  //! True if underlying map has been set, false otherwise.
  bool hasUnderlyingMap_;

  //! Pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
  kindr::HomTransformQuatD pose_;

  //! ROS publishers.
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr elevationMapFusedPublisher_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr visibilityCleanupMapPublisher_;

  //! Mutex lock for fused map.
  boost::recursive_mutex fusedMapMutex_;

  //! Mutex lock for raw map.
  boost::recursive_mutex rawMapMutex_;

  //! Mutex lock for visibility cleanup map.
  boost::recursive_mutex visibilityCleanupMapMutex_;

  //! Underlying map subscriber.
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr underlyingMapSubscriber_;

  //! Initial ros time
  rclcpp::Time initialTime_;

  //! Parameters. Are set through the ElevationMapping class.
  struct Parameters {
    double minVariance_{0.000009};
    double maxVariance_{0.0009};
    double mahalanobisDistanceThreshold_{2.5};
    double multiHeightNoise_{0.000009};
    double minHorizontalVariance_{0.0001};
    double maxHorizontalVariance_{0.05};
    std::string underlyingMapTopic_;
    bool enableVisibilityCleanup_{true};
    bool enableContinuousCleanup_{false};
    double visibilityCleanupDuration_{0.0};
    double scanningDuration_{1.0};
    double increaseHeightAlpha_{1.0};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;
};

}  // namespace elevation_mapping
