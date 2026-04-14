/*
 * ElevationMapping.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 * Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include <grid_map_msgs/srv/get_grid_map.hpp>
#include <grid_map_msgs/srv/process_file.hpp>
#include <grid_map_msgs/srv/set_grid_map.hpp>

// ROS
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <memory>
#include <thread>

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

enum class InitializationMethods { PlanarFloorInitializer };

/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */
class ElevationMapping : public rclcpp::Node {
 public:
  /*!
   * Constructor.
   */
  explicit ElevationMapping(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /*!
   * Destructor.
   */
  virtual ~ElevationMapping();

  /*!
   * Callback function for new data to be added to the elevation map.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointCloudMsg, bool publishPointCloud,
                          const SensorProcessorBase::Ptr& sensorProcessor_);

  /*!
   * Callback function for the update timer.
   */
  void mapUpdateTimerCallback();

  /*!
   * Callback function for the fused map publish timer.
   */
  void publishFusedMapCallback();

  /*!
   * Callback function for cleaning map based on visibility ray tracing.
   */
  void visibilityCleanupCallback();

  // Service callbacks
  void fuseEntireMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void getFusedSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                     std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response);

  void getRawSubmapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::GetGridMap::Request> request,
                                   std::shared_ptr<grid_map_msgs::srv::GetGridMap::Response> response);

  void enableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void disableUpdatesServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void clearMapServiceCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                               std::shared_ptr<std_srvs::srv::Empty::Response> response);

  void maskedReplaceServiceCallback(const std::shared_ptr<grid_map_msgs::srv::SetGridMap::Request> request,
                                    std::shared_ptr<grid_map_msgs::srv::SetGridMap::Response> response);

  void saveMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  void loadMapServiceCallback(const std::shared_ptr<grid_map_msgs::srv::ProcessFile::Request> request,
                              std::shared_ptr<grid_map_msgs::srv::ProcessFile::Response> response);

  void reloadParametersServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);

 private:
  bool readParameters(bool reload = false);
  bool initialize();
  void setupSubscribers();
  void setupServices();
  void setupTimers();
  void runFusionServiceThread();
  void visibilityCleanupThread();
  bool updatePrediction(const rclcpp::Time& time);
  bool updateMapLocation();
  void resetMapUpdateTimer();
  void stopMapUpdateTimer();
  bool initializeElevationMap();
  bool isFusingEnabled();

 protected:
  //! Input sources.
  InputSourceManager inputSources_;

  //! ROS subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudSubscriber_;
  message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped, rclcpp::Node> robotPoseSubscriber_;

  //! ROS service servers.
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr fusionTriggerService_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr fusedSubmapService_;
  rclcpp::Service<grid_map_msgs::srv::GetGridMap>::SharedPtr rawSubmapService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enableUpdatesService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disableUpdatesService_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clearMapService_;
  rclcpp::Service<grid_map_msgs::srv::SetGridMap>::SharedPtr maskedReplaceService_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr saveMapService_;
  rclcpp::Service<grid_map_msgs::srv::ProcessFile>::SharedPtr loadMapService_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reloadParametersService_;

  //! Callback groups for separate execution contexts.
  rclcpp::CallbackGroup::SharedPtr fusionCallbackGroup_;
  rclcpp::CallbackGroup::SharedPtr visibilityCleanupCallbackGroup_;

  //! Executors for callback groups.
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> fusionServiceExecutor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> visibilityCleanupExecutor_;

  //! Callback thread for the fusion services.
  std::thread fusionServiceThread_;

  //! Cache for the robot pose messages.
  message_filters::Cache<geometry_msgs::msg::PoseWithCovarianceStamped> robotPoseCache_;

  //! TF2 buffer and listener.
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  struct Parameters {
    //! Size of the cache for the robot pose messages.
    int robotPoseCacheSize_{200};

    //! Frame ID of the elevation map
    std::string mapFrameId_;

    //! Point which the elevation map follows.
    kindr::Position3D trackPoint_;
    std::string trackPointFrameId_;

    //! ROS topics for subscriptions.
    std::string pointCloudTopic_;
    std::string robotPoseTopic_;

    //! If true, robot motion updates are ignored.
    bool ignoreRobotMotionUpdates_{false};

    //! If false, elevation mapping stops updating
    bool updatesEnabled_{true};

    //! Maximum time that the map will not be updated (seconds).
    double maxNoUpdateDuration_{0.0};

    //! Time tolerance for updating the map with data before the last update (seconds).
    double timeTolerance_{0.0};

    //! Duration for the publishing the fusing map (seconds).
    double fusedMapPublishTimerDuration_{0.0};

    //! If map is fused after every change for debugging/analysis purposes.
    bool isContinuouslyFusing_{false};

    //! Duration for the raytracing cleanup timer (seconds).
    double visibilityCleanupTimerDuration_{0.0};

    //! Name of the mask layer used in the masked replace service
    std::string maskedReplaceServiceMaskLayerName_;

    //! Enables initialization of the elevation map
    bool initializeElevationMap_{false};

    //! Enum to choose the initialization method
    int initializationMethod_{0};

    //! Width of submap of the elevation map with a constant height
    double lengthInXInitSubmap_{1.2};

    //! Height of submap of the elevation map with a constant height
    double lengthInYInitSubmap_{1.8};

    //! Target frame to get the init height of the elevation map
    std::string targetFrameInitSubmap_;

    //! Additional offset of the height value
    double initSubmapHeightOffset_{0.0};

    //! Initial variance when setting a submap.
    double initSubmapVariance_{0.01};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  //! Elevation map.
  ElevationMap map_;

  //! Sensor processors. Deprecated use the one from input sources instead.
  SensorProcessorBase::Ptr sensorProcessor_;

  //! Robot motion elevation map updater.
  RobotMotionMapUpdater robotMotionMapUpdater_;

  //! Timer for the robot motion update.
  rclcpp::TimerBase::SharedPtr mapUpdateTimer_;

  //! Time of the last point cloud update.
  rclcpp::Time lastPointCloudUpdateTime_;

  //! Timer for publishing the fused map.
  rclcpp::TimerBase::SharedPtr fusedMapPublishTimer_;

  //! Timer for the raytracing cleanup.
  rclcpp::TimerBase::SharedPtr visibilityCleanupTimer_;

  //! Callback thread for raytracing cleanup.
  std::thread visibilityCleanupThread_;

  //! Becomes true when corresponding poses and point clouds can be found
  bool receivedFirstMatchingPointcloudAndPose_;
};

}  // namespace elevation_mapping
