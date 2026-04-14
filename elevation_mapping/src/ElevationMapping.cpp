/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <memory>
#include <string>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/recursive_mutex.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

ElevationMapping::ElevationMapping(const rclcpp::NodeOptions& options)
    : rclcpp::Node("elevation_mapping", options),
      inputSources_(this),
      map_(this),
      robotMotionMapUpdater_(this),
      lastPointCloudUpdateTime_(0, 0, RCL_ROS_TIME),
      receivedFirstMatchingPointcloudAndPose_(false) {
#ifndef NDEBUG
  RCLCPP_WARN(this->get_logger(), "CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  RCLCPP_INFO(this->get_logger(), "Elevation mapping node started.");

  // Create TF buffer and listener
  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Create callback groups for multi-threaded execution
  fusionCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  visibilityCleanupCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  readParameters();
  setupSubscribers();
  setupServices();
  setupTimers();

  initialize();

  RCLCPP_INFO(this->get_logger(), "Successfully launched node.");
}

void ElevationMapping::setupSubscribers() {
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
  const bool hasDeprecatedPointcloudTopic = this->has_parameter("point_cloud_topic");
  if (hasDeprecatedPointcloudTopic) {
    RCLCPP_WARN(this->get_logger(), "Parameter 'point_cloud_topic' is deprecated, please use 'input_sources' instead.");
  }
  if (!configuredInputSources && hasDeprecatedPointcloudTopic) {
    pointCloudSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        parameters.pointCloudTopic_, 1,
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) { pointCloudCallback(msg, true, sensorProcessor_); });
  }
  if (configuredInputSources) {
    inputSources_.registerCallbacks(*this, make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
  }

  if (!parameters.robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(this, parameters.robotPoseTopic_);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(parameters.robotPoseCacheSize_);
  } else {
    parameters.ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::setupServices() {
  fusionTriggerService_ = this->create_service<std_srvs::srv::Empty>(
      "trigger_fusion",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
             std::shared_ptr<std_srvs::srv::Empty::Response> res) { fuseEntireMapServiceCallback(req, res); },
      rmw_qos_profile_services_default, fusionCallbackGroup_);

  fusedSubmapService_ = this->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_submap",
      [this](const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> req,
             std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> res) { getFusedSubmapServiceCallback(req, res); },
      rmw_qos_profile_services_default, fusionCallbackGroup_);

  rawSubmapService_ = this->create_service<grid_map_msgs::srv::GetGridMap>(
      "get_raw_submap",
      [this](const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> req,
             std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> res) { getRawSubmapServiceCallback(req, res); },
      rmw_qos_profile_services_default, fusionCallbackGroup_);

  clearMapService_ = this->create_service<std_srvs::srv::Empty>(
      "clear_map", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                          std::shared_ptr<std_srvs::srv::Empty::Response> res) { clearMapServiceCallback(req, res); });

  enableUpdatesService_ = this->create_service<std_srvs::srv::Empty>(
      "enable_updates", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                               std::shared_ptr<std_srvs::srv::Empty::Response> res) { enableUpdatesServiceCallback(req, res); });

  disableUpdatesService_ = this->create_service<std_srvs::srv::Empty>(
      "disable_updates", [this](const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                std::shared_ptr<std_srvs::srv::Empty::Response> res) { disableUpdatesServiceCallback(req, res); });

  maskedReplaceService_ = this->create_service<grid_map_msgs::srv::SetGridMap>(
      "masked_replace",
      [this](const std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> req,
             std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> res) { maskedReplaceServiceCallback(req, res); });

  saveMapService_ = this->create_service<grid_map_msgs::srv::ProcessFile>(
      "save_map", [this](const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> req,
                         std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> res) { saveMapServiceCallback(req, res); });

  loadMapService_ = this->create_service<grid_map_msgs::srv::ProcessFile>(
      "load_map", [this](const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> req,
                         std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> res) { loadMapServiceCallback(req, res); });

  reloadParametersService_ = this->create_service<std_srvs::srv::Trigger>(
      "reload_parameters",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) { reloadParametersServiceCallback(req, res); });
}

void ElevationMapping::setupTimers() {
  const Parameters parameters{parameters_.getData()};
  ElevationMap::Parameters mapParameters{map_.parameters_.getData()};

  if (parameters.maxNoUpdateDuration_ > 0.0) {
    mapUpdateTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(parameters.maxNoUpdateDuration_), [this]() { mapUpdateTimerCallback(); },
        fusionCallbackGroup_);
    mapUpdateTimer_->cancel();
  }

  if (parameters.fusedMapPublishTimerDuration_ > 0.0) {
    fusedMapPublishTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(parameters.fusedMapPublishTimerDuration_), [this]() { publishFusedMapCallback(); },
        fusionCallbackGroup_);
  }

  if (mapParameters.enableVisibilityCleanup_ && parameters.visibilityCleanupTimerDuration_ > 0.0 &&
      !mapParameters.enableContinuousCleanup_) {
    visibilityCleanupTimer_ = this->create_wall_timer(
        std::chrono::duration<double>(parameters.visibilityCleanupTimerDuration_), [this]() { visibilityCleanupCallback(); },
        visibilityCleanupCallbackGroup_);
  }
}

ElevationMapping::~ElevationMapping() {
  if (fusionServiceExecutor_) {
    fusionServiceExecutor_->cancel();
  }
  if (visibilityCleanupExecutor_) {
    visibilityCleanupExecutor_->cancel();
  }
  if (fusionServiceThread_.joinable()) {
    fusionServiceThread_.join();
  }
  if (visibilityCleanupThread_.joinable()) {
    visibilityCleanupThread_.join();
  }
}

bool ElevationMapping::readParameters(bool reload) {
  auto [parameters, parametersGuard] = parameters_.getDataToWrite();
  auto [mapParameters, mapParametersGuard] = map_.parameters_.getDataToWrite();

  auto declareAndGet = [&](const std::string& name, auto defaultVal) {
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, defaultVal);
    }
    using T = decltype(defaultVal);
    return this->get_parameter(name).get_value<T>();
  };

  parameters.pointCloudTopic_ = declareAndGet("point_cloud_topic", std::string("/points"));
  parameters.robotPoseTopic_ = declareAndGet("robot_pose_with_covariance_topic", std::string("/pose"));
  parameters.trackPointFrameId_ = declareAndGet("track_point_frame_id", std::string("/robot"));
  parameters.trackPoint_.x() = declareAndGet("track_point_x", 0.0);
  parameters.trackPoint_.y() = declareAndGet("track_point_y", 0.0);
  parameters.trackPoint_.z() = declareAndGet("track_point_z", 0.0);

  parameters.robotPoseCacheSize_ = declareAndGet("robot_pose_cache_size", 200);
  if (parameters.robotPoseCacheSize_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "robot_pose_cache_size must be non-negative.");
    return false;
  }

  double minUpdateRate = declareAndGet("min_update_rate", 2.0);
  if (minUpdateRate == 0.0) {
    parameters.maxNoUpdateDuration_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Rate for publishing the map is zero.");
  } else {
    parameters.maxNoUpdateDuration_ = 1.0 / minUpdateRate;
  }
  if (parameters.maxNoUpdateDuration_ == 0.0) {
    RCLCPP_ERROR(this->get_logger(), "maxNoUpdateDuration is zero.");
    return false;
  }

  double timeTolerance = declareAndGet("time_tolerance", 0.0);
  parameters.timeTolerance_ = timeTolerance;

  double fusedMapPublishingRate = declareAndGet("fused_map_publishing_rate", 1.0);
  if (fusedMapPublishingRate == 0.0) {
    parameters.fusedMapPublishTimerDuration_ = 0.0;
    RCLCPP_WARN(this->get_logger(),
                "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service "
                "`triggerFusion` is called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    parameters.isContinuouslyFusing_ = true;
    parameters.fusedMapPublishTimerDuration_ = 0.0;
  } else {
    parameters.fusedMapPublishTimerDuration_ = 1.0 / fusedMapPublishingRate;
  }

  double visibilityCleanupRate = declareAndGet("visibility_cleanup_rate", 1.0);
  if (visibilityCleanupRate == 0.0) {
    parameters.visibilityCleanupTimerDuration_ = 0.0;
    RCLCPP_WARN(this->get_logger(), "Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    parameters.visibilityCleanupTimerDuration_ = 1.0 / visibilityCleanupRate;
    mapParameters.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  parameters.mapFrameId_ = declareAndGet("map_frame_id", std::string("/map"));

  grid_map::Length length;
  grid_map::Position position;
  double resolution{0.01};
  length(0) = declareAndGet("length_in_x", 1.5);
  length(1) = declareAndGet("length_in_y", 1.5);
  position.x() = declareAndGet("position_x", 0.0);
  position.y() = declareAndGet("position_y", 0.0);
  resolution = declareAndGet("resolution", resolution);

  if (!reload) {
    map_.setFrameId(parameters.mapFrameId_);
    map_.setGeometry(length, resolution, position);
  }

  mapParameters.minVariance_ = declareAndGet("min_variance", pow(0.003, 2));
  mapParameters.maxVariance_ = declareAndGet("max_variance", pow(0.03, 2));
  mapParameters.mahalanobisDistanceThreshold_ = declareAndGet("mahalanobis_distance_threshold", 2.5);
  mapParameters.multiHeightNoise_ = declareAndGet("multi_height_noise", pow(0.003, 2));
  mapParameters.minHorizontalVariance_ = declareAndGet("min_horizontal_variance", pow(resolution / 2.0, 2));
  mapParameters.maxHorizontalVariance_ = declareAndGet("max_horizontal_variance", 0.5);
  mapParameters.underlyingMapTopic_ = declareAndGet("underlying_map_topic", std::string());
  mapParameters.enableVisibilityCleanup_ = declareAndGet("enable_visibility_cleanup", true);
  mapParameters.enableContinuousCleanup_ = declareAndGet("enable_continuous_cleanup", false);
  mapParameters.scanningDuration_ = declareAndGet("scanning_duration", 1.0);
  mapParameters.increaseHeightAlpha_ = declareAndGet("increase_height_alpha", 0.0);
  parameters.maskedReplaceServiceMaskLayerName_ = declareAndGet("masked_replace_service_mask_layer_name", std::string("mask"));

  parameters.initializeElevationMap_ = declareAndGet("initialize_elevation_map", false);
  parameters.initializationMethod_ = declareAndGet("initialization_method", 0);
  parameters.lengthInXInitSubmap_ = declareAndGet("length_in_x_init_submap", 1.2);
  parameters.lengthInYInitSubmap_ = declareAndGet("length_in_y_init_submap", 1.8);
  parameters.initSubmapHeightOffset_ = declareAndGet("init_submap_height_offset", 0.0);
  parameters.initSubmapVariance_ = declareAndGet("init_submap_variance", 0.01);
  parameters.targetFrameInitSubmap_ = declareAndGet("target_frame_init_submap", std::string("/footprint"));

  std::string sensorType = declareAndGet("sensor_processor/type", std::string("structured_light"));

  std::string robotBaseFrameId = declareAndGet("robot_base_frame_id", std::string("/robot"));
  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{robotBaseFrameId, parameters.mapFrameId_};

  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(this, generalSensorProcessorConfig, tfBuffer_);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(this, generalSensorProcessorConfig, tfBuffer_);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(this, generalSensorProcessorConfig, tfBuffer_);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(this, generalSensorProcessorConfig, tfBuffer_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "The sensor type %s is not available.", sensorType.c_str());
  }
  if (sensorProcessor_ && !sensorProcessor_->readParameters()) {
    return false;
  }
  if (!robotMotionMapUpdater_.readParameters()) {
    return false;
  }

  return true;
}

bool ElevationMapping::initialize() {
  RCLCPP_INFO(this->get_logger(), "Elevation mapping node initializing ... ");
  fusionServiceThread_ = std::thread(&ElevationMapping::runFusionServiceThread, this);
  rclcpp::sleep_for(std::chrono::seconds(1));
  resetMapUpdateTimer();
  visibilityCleanupThread_ = std::thread([this] { visibilityCleanupThread(); });
  initializeElevationMap();
  return true;
}

void ElevationMapping::runFusionServiceThread() {
  fusionServiceExecutor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  fusionServiceExecutor_->add_callback_group(fusionCallbackGroup_, this->get_node_base_interface());
  fusionServiceExecutor_->spin();
}

void ElevationMapping::visibilityCleanupThread() {
  visibilityCleanupExecutor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  visibilityCleanupExecutor_->add_callback_group(visibilityCleanupCallbackGroup_, this->get_node_base_interface());
  visibilityCleanupExecutor_->spin();
}

void ElevationMapping::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg, bool publishPointCloud,
                                          const SensorProcessorBase::Ptr& sensorProcessor_) {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_DEBUG(this->get_logger(), "Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!parameters.updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_.setTimestamp(this->now());
      map_.postprocessAndPublishRawElevationMap();
    }
    return;
  }

  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().seconds();
    const double currentPointCloudTime = rclcpp::Time(pointCloudMsg->header.stamp).seconds();

    if (currentPointCloudTime < oldestPoseTime) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  stopMapUpdateTimer();

  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_ = rclcpp::Time(static_cast<int64_t>(pointCloud->header.stamp) * 1000LL, RCL_ROS_TIME);

  RCLCPP_DEBUG(this->get_logger(), "ElevationMap received a point cloud (%i points) for elevation mapping.",
               static_cast<int>(pointCloud->size()));

  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!parameters.ignoreRobotMotionUpdates_) {
    auto poseMessage = robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
        RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f",
                     robotPoseCache_.getOldestTime().seconds(), lastPointCloudUpdateTime_.seconds());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?",
                     lastPointCloudUpdateTime_.seconds());
      }
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                  pointCloudMsg->header.frame_id)) {
    if (!sensorProcessor_->isTfAvailableInBuffer()) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                           "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "Point cloud could not be processed. (Throttled 10s)");
    resetMapUpdateTimer();
    return;
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  updateMapLocation();

  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    RCLCPP_ERROR(this->get_logger(), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  ElevationMap::Parameters mapParameters{map_.parameters_.getData()};

  if (mapParameters.enableContinuousCleanup_) {
    RCLCPP_DEBUG(this->get_logger(), "Clearing elevation map before adding new point cloud.");
    map_.clear();
  }

  if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    RCLCPP_ERROR(this->get_logger(), "Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    map_.postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      map_.fuseAll();
      map_.publishFusedElevationMap();
    }
  }

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback() {
  const Parameters parameters{parameters_.getData()};
  if (!parameters.updatesEnabled_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                         "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_.setTimestamp(this->now());
    map_.postprocessAndPublishRawElevationMap();
    return;
  }

  rclcpp::Time time = this->now();
  double periodSinceLastUpdate = (time - lastPointCloudUpdateTime_).seconds();
  if (periodSinceLastUpdate <= parameters.maxNoUpdateDuration_) {
    return;
  }
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();

  if (!updatePrediction(time)) {
    RCLCPP_ERROR(this->get_logger(), "Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  map_.postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback() {
  if (!map_.hasFusedMapSubscribers()) {
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback() {
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is running visibility cleanup.");
  map_.visibilityCleanup(lastPointCloudUpdateTime_);
}

void ElevationMapping::fuseEntireMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

bool ElevationMapping::isFusingEnabled() {
  const Parameters parameters{parameters_.getData()};
  return parameters.isContinuouslyFusing_ && map_.hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const rclcpp::Time& time) {
  const Parameters parameters{parameters_.getData()};
  if (parameters.ignoreRobotMotionUpdates_) {
    return true;
  }

  RCLCPP_DEBUG(this->get_logger(), "Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().seconds());

  if ((time + rclcpp::Duration::from_seconds(parameters.timeTolerance_)) < map_.getTimeOfLastUpdate()) {
    RCLCPP_ERROR(this->get_logger(), "Requested update with time stamp %f, but time of last update was %f.", time.seconds(),
                 map_.getTimeOfLastUpdate().seconds());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    RCLCPP_DEBUG(this->get_logger(), "Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.seconds(),
                 map_.getTimeOfLastUpdate().seconds());
    return true;
  }

  auto poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    if (robotPoseCache_.getOldestTime().seconds() > lastPointCloudUpdateTime_.seconds()) {
      RCLCPP_ERROR(this->get_logger(), "The oldest pose available is at %f, requested pose at %f",
                   robotPoseCache_.getOldestTime().seconds(), lastPointCloudUpdateTime_.seconds());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Could not get pose information from robot for time %f. Buffer empty?",
                   lastPointCloudUpdateTime_.seconds());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation() {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_DEBUG(this->get_logger(), "Elevation map is checked for relocalization.");

  try {
    // Look up transform from trackPointFrameId to map frame
    geometry_msgs::msg::TransformStamped transformMsg =
        tfBuffer_->lookupTransform(map_.getFrameId(), parameters.trackPointFrameId_, tf2::TimePointZero);

    // Build track point in the map frame by applying transform to the trackPoint
    geometry_msgs::msg::PointStamped trackPointIn;
    trackPointIn.header.frame_id = parameters.trackPointFrameId_;
    trackPointIn.header.stamp = rclcpp::Time(0);
    kindr_ros::convertToRosGeometryMsg(parameters.trackPoint_, trackPointIn.point);

    geometry_msgs::msg::PointStamped trackPointOut;
    tf2::doTransform(trackPointIn, trackPointOut, transformMsg);

    kindr::Position3D position3d;
    kindr_ros::convertFromRosGeometryMsg(trackPointOut.point, position3d);
    grid_map::Position position = position3d.vector().head(2);
    map_.move(position);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return false;
  }

  return true;
}

void ElevationMapping::getFusedSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
               requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request->layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response->map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response->map);
  }

  RCLCPP_DEBUG(this->get_logger(), "Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().seconds());
}

void ElevationMapping::getRawSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response) {
  grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
  grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
  RCLCPP_DEBUG(this->get_logger(), "Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
               requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request->layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response->map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request->layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response->map);
  }
}

void ElevationMapping::disableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                                     std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Disabling updates.");
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  parameters.updatesEnabled_ = false;
}

void ElevationMapping::enableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                                    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Enabling updates.");
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  parameters.updatesEnabled_ = true;
}

bool ElevationMapping::initializeElevationMap() {
  const Parameters parameters{parameters_.getData()};
  if (parameters.initializeElevationMap_) {
    if (static_cast<elevation_mapping::InitializationMethods>(parameters.initializationMethod_) ==
        elevation_mapping::InitializationMethods::PlanarFloorInitializer) {
      try {
        geometry_msgs::msg::TransformStamped transformMsg =
            tfBuffer_->lookupTransform(parameters.mapFrameId_, parameters.targetFrameInitSubmap_, tf2::TimePointZero,
                                       tf2::durationFromSec(5.0));

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing with x: " << transformMsg.transform.translation.x
                                                                         << " y: " << transformMsg.transform.translation.y
                                                                         << " z: " << transformMsg.transform.translation.z);

        const grid_map::Position positionRobot(transformMsg.transform.translation.x, transformMsg.transform.translation.y);

        map_.move(positionRobot);

        map_.setRawSubmapHeight(positionRobot,
                                static_cast<float>(transformMsg.transform.translation.z + parameters.initSubmapHeightOffset_),
                                static_cast<float>(parameters.initSubmapVariance_), parameters.lengthInXInitSubmap_,
                                parameters.lengthInYInitSubmap_);
        return true;
      } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG(this->get_logger(), "%s", ex.what());
        RCLCPP_WARN(this->get_logger(),
                    "Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }
  return true;
}

void ElevationMapping::clearMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
                                               std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/) {
  RCLCPP_INFO(this->get_logger(), "Clearing map...");
  map_.clear();
  initializeElevationMap();
  RCLCPP_INFO(this->get_logger(), "Map cleared.");
}

void ElevationMapping::maskedReplaceServiceCallback(const std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                                    std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> /*response*/) {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_INFO(this->get_logger(), "Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request->map, sourceMap);

  grid_map::Matrix mask;
  if (sourceMap.exists(parameters.maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[parameters.maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  boost::recursive_mutex::scoped_lock scopedLockRawData(map_.getRawDataMutex());

  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    if (*sourceLayerIterator == parameters.maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    if (map_.getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_.getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_.getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_.getRawGridMap().getPosition(*destinationIterator, position);

        if (!sourceMap.isInside(position)) {
          continue;
        }

        grid_map::Index sourceIndex;
        sourceMap.getIndex(position, sourceIndex);
        if (!std::isnan(mask(sourceIndex(0), sourceIndex(1)))) {
          destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }
}

void ElevationMapping::saveMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> /*request*/,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_WARN(this->get_logger(), "save_map service: saveToBag is not supported in ROS2. Returning failure.");
  response->success = static_cast<unsigned char>(false);
}

void ElevationMapping::loadMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> /*request*/,
                                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response) {
  RCLCPP_WARN(this->get_logger(), "load_map service: loadFromBag is not supported in ROS2. Returning failure.");
  response->success = static_cast<unsigned char>(false);
}

void ElevationMapping::reloadParametersServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  Parameters fallbackParameters{parameters_.getData()};

  const bool success{readParameters(true)};
  response->success = success;

  if (success) {
    response->message = "Successfully reloaded parameters!";
  } else {
    parameters_.setData(fallbackParameters);
    response->message = "Reloading parameters failed, reverted parameters to previous state!";
  }
}

void ElevationMapping::resetMapUpdateTimer() {
  const Parameters parameters{parameters_.getData()};
  if (!mapUpdateTimer_) {
    return;
  }
  mapUpdateTimer_->cancel();
  double periodSinceLastUpdate = (this->now() - map_.getTimeOfLastUpdate()).seconds();
  if (periodSinceLastUpdate > parameters.maxNoUpdateDuration_) {
    periodSinceLastUpdate = 0.0;
  }
  double newPeriod = parameters.maxNoUpdateDuration_ - periodSinceLastUpdate;
  mapUpdateTimer_ = this->create_wall_timer(std::chrono::duration<double>(newPeriod), [this]() { mapUpdateTimerCallback(); },
                                            fusionCallbackGroup_);
}

void ElevationMapping::stopMapUpdateTimer() {
  if (mapUpdateTimer_) {
    mapUpdateTimer_->cancel();
  }
}

}  // namespace elevation_mapping
