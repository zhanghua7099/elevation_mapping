/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 * Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <cstring>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

namespace {
float intAsFloat(const uint32_t input) {
  float output{std::nanf("1")};
  std::memcpy(&output, &input, sizeof(uint32_t));
  return output;
}
}  // namespace

namespace elevation_mapping {

ElevationMap::ElevationMap(rclcpp::Node* node)
    : node_(node),
      rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "horizontal_variance_xy", "color", "time",
               "dynamic_time", "lowest_scan_point", "sensor_x_at_lowest_scan", "sensor_y_at_lowest_scan", "sensor_z_at_lowest_scan"}),
      fusedMap_({"elevation", "upper_bound", "lower_bound", "color"}),
      postprocessorPool_([node]() -> int {
        if (!node->has_parameter("postprocessor_num_threads")) {
          node->declare_parameter("postprocessor_num_threads", 1);
        }
        return node->get_parameter("postprocessor_num_threads").as_int();
      }(), node),
      hasUnderlyingMap_(false),
      initialTime_(0, 0, RCL_ROS_TIME) {
  rawMap_.setBasicLayers({"elevation", "variance"});
  fusedMap_.setBasicLayers({"elevation", "upper_bound", "lower_bound"});
  clear();
  const Parameters parameters{parameters_.getData()};

  elevationMapFusedPublisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("elevation_map", 1);
  if (!parameters.underlyingMapTopic_.empty()) {
    underlyingMapSubscriber_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
        parameters.underlyingMapTopic_, 1,
        [this](const grid_map_msgs::msg::GridMap::ConstSharedPtr& msg) { underlyingMapCallback(msg); });
  }
  visibilityCleanupMapPublisher_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("visibility_cleanup_map", 1);

  initialTime_ = node_->now();
}

ElevationMap::~ElevationMap() = default;

void ElevationMap::setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position) {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  rawMap_.setGeometry(length, resolution, position);
  fusedMap_.setGeometry(length, resolution, position);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Elevation map grid resized to " << rawMap_.getSize()(0) << " rows and " << rawMap_.getSize()(1) << " columns.");
}

bool ElevationMap::add(const PointCloudType::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances, const rclcpp::Time& timestamp,
                       const Eigen::Affine3d& transformationSensorToMap) {
  const Parameters parameters{parameters_.getData()};
  if (static_cast<unsigned int>(pointCloud->size()) != static_cast<unsigned int>(pointCloudVariances.size())) {
    RCLCPP_ERROR(node_->get_logger(), "ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.",
                 (int)pointCloud->size(), (int)pointCloudVariances.size());
    return false;
  }

  const auto methodStartTime = std::chrono::steady_clock::now();
  const rclcpp::Time currentTime = node_->now();
  const float currentTimeSecondsPattern{intAsFloat(static_cast<uint32_t>(static_cast<uint64_t>(currentTime.seconds())))};
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  if (initialTime_.nanoseconds() == 0) {
    initialTime_ = timestamp;
  }
  const float scanTimeSinceInitialization = static_cast<float>((timestamp - initialTime_).seconds());

  auto& elevationLayer = rawMap_["elevation"];
  auto& varianceLayer = rawMap_["variance"];
  auto& horizontalVarianceXLayer = rawMap_["horizontal_variance_x"];
  auto& horizontalVarianceYLayer = rawMap_["horizontal_variance_y"];
  auto& horizontalVarianceXYLayer = rawMap_["horizontal_variance_xy"];
  auto& colorLayer = rawMap_["color"];
  auto& timeLayer = rawMap_["time"];
  auto& dynamicTimeLayer = rawMap_["dynamic_time"];
  auto& lowestScanPointLayer = rawMap_["lowest_scan_point"];
  auto& sensorXatLowestScanLayer = rawMap_["sensor_x_at_lowest_scan"];
  auto& sensorYatLowestScanLayer = rawMap_["sensor_y_at_lowest_scan"];
  auto& sensorZatLowestScanLayer = rawMap_["sensor_z_at_lowest_scan"];

  std::vector<Eigen::Ref<const grid_map::Matrix>> basicLayers_;
  for (const std::string& layer : rawMap_.getBasicLayers()) {
    basicLayers_.emplace_back(rawMap_.get(layer));
  }

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    auto& point = pointCloud->points[i];
    grid_map::Index index;
    grid_map::Position position(point.x, point.y);
    if (!rawMap_.getIndex(position, index)) {
      continue;
    }

    auto& elevation = elevationLayer(index(0), index(1));
    auto& variance = varianceLayer(index(0), index(1));
    auto& horizontalVarianceX = horizontalVarianceXLayer(index(0), index(1));
    auto& horizontalVarianceY = horizontalVarianceYLayer(index(0), index(1));
    auto& horizontalVarianceXY = horizontalVarianceXYLayer(index(0), index(1));
    auto& color = colorLayer(index(0), index(1));
    auto& time = timeLayer(index(0), index(1));
    auto& dynamicTime = dynamicTimeLayer(index(0), index(1));
    auto& lowestScanPoint = lowestScanPointLayer(index(0), index(1));
    auto& sensorXatLowestScan = sensorXatLowestScanLayer(index(0), index(1));
    auto& sensorYatLowestScan = sensorYatLowestScanLayer(index(0), index(1));
    auto& sensorZatLowestScan = sensorZatLowestScanLayer(index(0), index(1));

    const float& pointVariance = pointCloudVariances(i);
    bool isValid = std::all_of(basicLayers_.begin(), basicLayers_.end(),
                               [&](Eigen::Ref<const grid_map::Matrix> layer) { return std::isfinite(layer(index(0), index(1))); });
    if (!isValid) {
      elevation = point.z;
      variance = pointVariance;
      horizontalVarianceX = parameters.minHorizontalVariance_;
      horizontalVarianceY = parameters.minHorizontalVariance_;
      horizontalVarianceXY = 0.0;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    const double mahalanobisDistance = fabs(point.z - elevation) / sqrt(variance);
    if (mahalanobisDistance > parameters.mahalanobisDistanceThreshold_) {
      if (scanTimeSinceInitialization - time <= parameters.scanningDuration_ && elevation > point.z) {
        // Ignore point
      } else if (scanTimeSinceInitialization - time <= parameters.scanningDuration_) {
        elevation = parameters.increaseHeightAlpha_ * elevation + (1.0 - parameters.increaseHeightAlpha_) * point.z;
        variance = parameters.increaseHeightAlpha_ * variance + (1.0 - parameters.increaseHeightAlpha_) * pointVariance;
      } else {
        variance += parameters.multiHeightNoise_;
      }
      continue;
    }

    const float pointHeightPlusUncertainty = point.z + 3.0 * sqrt(pointVariance);
    if (std::isnan(lowestScanPoint) || pointHeightPlusUncertainty < lowestScanPoint) {
      lowestScanPoint = pointHeightPlusUncertainty;
      const grid_map::Position3 sensorTranslation(transformationSensorToMap.translation());
      sensorXatLowestScan = sensorTranslation.x();
      sensorYatLowestScan = sensorTranslation.y();
      sensorZatLowestScan = sensorTranslation.z();
    }

    elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
    variance = (pointVariance * variance) / (pointVariance + variance);
    grid_map::colorVectorToValue(point.getRGBVector3i(), color);
    time = scanTimeSinceInitialization;
    dynamicTime = currentTimeSecondsPattern;

    horizontalVarianceX = parameters.minHorizontalVariance_;
    horizontalVarianceY = parameters.minHorizontalVariance_;
    horizontalVarianceXY = 0.0;
  }

  clean();
  rawMap_.setTimestamp(timestamp.nanoseconds());

  const auto duration = std::chrono::steady_clock::now() - methodStartTime;
  RCLCPP_DEBUG(node_->get_logger(), "Raw map has been updated with a new point cloud in %f s.",
               std::chrono::duration<double>(duration).count());
  return true;
}

bool ElevationMap::update(const grid_map::Matrix& varianceUpdate, const grid_map::Matrix& horizontalVarianceUpdateX,
                          const grid_map::Matrix& horizontalVarianceUpdateY, const grid_map::Matrix& horizontalVarianceUpdateXY,
                          const rclcpp::Time& time) {
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

  const auto& size = rawMap_.getSize();

  if (!((grid_map::Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all() &&
        (grid_map::Index(horizontalVarianceUpdateXY.rows(), horizontalVarianceUpdateXY.cols()) == size).all())) {
    RCLCPP_ERROR(node_->get_logger(), "The size of the update matrices does not match.");
    return false;
  }

  rawMap_.get("variance") += varianceUpdate;
  rawMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
  rawMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
  rawMap_.get("horizontal_variance_xy") += horizontalVarianceUpdateXY;
  clean();
  rawMap_.setTimestamp(time.nanoseconds());

  return true;
}

bool ElevationMap::fuseAll() {
  RCLCPP_DEBUG(node_->get_logger(), "Requested to fuse entire elevation map.");
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return fuse(grid_map::Index(0, 0), fusedMap_.getSize());
}

bool ElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length) {
  RCLCPP_DEBUG(node_->get_logger(),
               "Requested to fuse an area of the elevation map with center at (%f, %f) and side lengths (%f, %f)", position[0],
               position[1], length[0], length[1]);

  grid_map::Index topLeftIndex;
  grid_map::Index submapBufferSize;
  grid_map::Position submapPosition;
  grid_map::Length submapLength;
  grid_map::Index requestedIndexInSubmap;

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  grid_map::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap, position, length,
                                 rawMap_.getLength(), rawMap_.getPosition(), rawMap_.getResolution(), rawMap_.getSize(),
                                 rawMap_.getStartIndex());

  return fuse(topLeftIndex, submapBufferSize);
}

bool ElevationMap::fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size) {
  RCLCPP_DEBUG(node_->get_logger(), "Fusing elevation map...");

  if ((size == 0).any()) {
    return false;
  }

  const auto methodStartTime = std::chrono::steady_clock::now();

  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  auto rawMapCopy = rawMap_;
  scopedLockForRawData.unlock();

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  const double halfResolution = fusedMap_.getResolution() / 2.0;
  const float minimalWeight = std::numeric_limits<float>::epsilon() * static_cast<float>(2.0);
  const double ellipseExtension = M_SQRT2 * fusedMap_.getResolution();

  if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) {
    resetFusedData();
  }

  if (rawMapCopy.getPosition() != fusedMap_.getPosition()) {
    fusedMap_.move(rawMapCopy.getPosition());
  }

  for (grid_map::SubmapIterator areaIterator(rawMapCopy, topLeftIndex, size); !areaIterator.isPastEnd(); ++areaIterator) {
    if (fusedMap_.isValid(*areaIterator)) {
      continue;
    }

    if (!rawMapCopy.isValid(*areaIterator)) {
      continue;
    }

    const float& sigmaXsquare = rawMapCopy.at("horizontal_variance_x", *areaIterator);
    const float& sigmaYsquare = rawMapCopy.at("horizontal_variance_y", *areaIterator);
    const float& sigmaXYsquare = rawMapCopy.at("horizontal_variance_xy", *areaIterator);

    Eigen::Matrix2d covarianceMatrix;
    covarianceMatrix << sigmaXsquare, sigmaXYsquare, sigmaXYsquare, sigmaYsquare;
    const double uncertaintyFactor = 2.486;
    Eigen::EigenSolver<Eigen::Matrix2d> solver(covarianceMatrix);
    Eigen::Array2d eigenvalues(solver.eigenvalues().real().cwiseAbs());

    Eigen::Array2d::Index maxEigenvalueIndex{0};
    eigenvalues.maxCoeff(&maxEigenvalueIndex);
    Eigen::Array2d::Index minEigenvalueIndex{0};
    maxEigenvalueIndex == Eigen::Array2d::Index(0) ? minEigenvalueIndex = 1 : minEigenvalueIndex = 0;
    const grid_map::Length ellipseLength =
        2.0 * uncertaintyFactor * grid_map::Length(eigenvalues(maxEigenvalueIndex), eigenvalues(minEigenvalueIndex)).sqrt() +
        ellipseExtension;
    const double ellipseRotation(
        atan2(solver.eigenvectors().col(maxEigenvalueIndex).real()(1), solver.eigenvectors().col(maxEigenvalueIndex).real()(0)));

    grid_map::Position requestedSubmapPosition;
    rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);
    grid_map::EllipseIterator ellipseIterator(rawMapCopy, requestedSubmapPosition, ellipseLength, ellipseRotation);

    Eigen::ArrayXf means;
    Eigen::ArrayXf weights;
    const unsigned int maxNumberOfCellsToFuse = ellipseIterator.getSubmapSize().prod();
    means.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);
    WeightedEmpiricalCumulativeDistributionFunction<float> lowerBoundDistribution;
    WeightedEmpiricalCumulativeDistributionFunction<float> upperBoundDistribution;

    float maxStandardDeviation = sqrt(eigenvalues(maxEigenvalueIndex));
    float minStandardDeviation = sqrt(eigenvalues(minEigenvalueIndex));
    Eigen::Rotation2Dd rotationMatrix(ellipseRotation);
    std::string maxEigenvalueLayer;
    std::string minEigenvalueLayer;
    if (maxEigenvalueIndex == 0) {
      maxEigenvalueLayer = "horizontal_variance_x";
      minEigenvalueLayer = "horizontal_variance_y";
    } else {
      maxEigenvalueLayer = "horizontal_variance_y";
      minEigenvalueLayer = "horizontal_variance_x";
    }

    size_t i = 0;
    for (; !ellipseIterator.isPastEnd(); ++ellipseIterator) {
      if (!rawMapCopy.isValid(*ellipseIterator)) {
        continue;
      }

      means[i] = rawMapCopy.at("elevation", *ellipseIterator);

      grid_map::Position absolutePosition;
      rawMapCopy.getPosition(*ellipseIterator, absolutePosition);
      Eigen::Vector2d distanceToCenter = (rotationMatrix * (absolutePosition - requestedSubmapPosition)).cwiseAbs();

      float probability1 = cumulativeDistributionFunction(distanceToCenter.x() + halfResolution, 0.0, maxStandardDeviation) -
                           cumulativeDistributionFunction(distanceToCenter.x() - halfResolution, 0.0, maxStandardDeviation);
      float probability2 = cumulativeDistributionFunction(distanceToCenter.y() + halfResolution, 0.0, minStandardDeviation) -
                           cumulativeDistributionFunction(distanceToCenter.y() - halfResolution, 0.0, minStandardDeviation);

      const float weight = std::max(minimalWeight, probability1 * probability2);
      weights[i] = weight;
      const float standardDeviation = sqrt(rawMapCopy.at("variance", *ellipseIterator));
      lowerBoundDistribution.add(means[i] - 2.0 * standardDeviation, weight);
      upperBoundDistribution.add(means[i] + 2.0 * standardDeviation, weight);

      i++;
    }

    if (i == 0) {
      fusedMap_.at("elevation", *areaIterator) = rawMapCopy.at("elevation", *areaIterator);
      fusedMap_.at("lower_bound", *areaIterator) =
          rawMapCopy.at("elevation", *areaIterator) - 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("upper_bound", *areaIterator) =
          rawMapCopy.at("elevation", *areaIterator) + 2.0 * sqrt(rawMapCopy.at("variance", *areaIterator));
      fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
      continue;
    }

    means.conservativeResize(i);
    weights.conservativeResize(i);

    float mean = (weights * means).sum() / weights.sum();

    if (!std::isfinite(mean)) {
      RCLCPP_ERROR(node_->get_logger(), "Something went wrong when fusing the map: Mean = %f", mean);
      continue;
    }

    fusedMap_.at("elevation", *areaIterator) = mean;
    lowerBoundDistribution.compute();
    upperBoundDistribution.compute();
    fusedMap_.at("lower_bound", *areaIterator) = lowerBoundDistribution.quantile(0.01);
    fusedMap_.at("upper_bound", *areaIterator) = upperBoundDistribution.quantile(0.99);
    fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
  }

  fusedMap_.setTimestamp(rawMapCopy.getTimestamp());

  const auto duration = std::chrono::steady_clock::now() - methodStartTime;
  RCLCPP_DEBUG(node_->get_logger(), "Elevation map has been fused in %f s.", std::chrono::duration<double>(duration).count());

  return true;
}

bool ElevationMap::clear() {
  {
    boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
    rawMap_.clearAll();
    rawMap_.resetTimestamp();
    rawMap_.get("dynamic_time").setZero();
  }
  {
    boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
    fusedMap_.clearAll();
    fusedMap_.resetTimestamp();
  }
  return true;
}

void ElevationMap::visibilityCleanup(const rclcpp::Time& updatedTime) {
  const Parameters parameters{parameters_.getData()};
  const auto methodStartTime = std::chrono::steady_clock::now();
  const double timeSinceInitialization = (updatedTime - initialTime_).seconds();

  boost::recursive_mutex::scoped_lock scopedLockForVisibilityCleanupData(visibilityCleanupMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  visibilityCleanupMap_ = rawMap_;
  rawMap_.clear("lowest_scan_point");
  rawMap_.clear("sensor_x_at_lowest_scan");
  rawMap_.clear("sensor_y_at_lowest_scan");
  rawMap_.clear("sensor_z_at_lowest_scan");
  scopedLockForRawData.unlock();
  visibilityCleanupMap_.add("max_height");

  for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) {
      continue;
    }
    const auto& lowestScanPoint = visibilityCleanupMap_.at("lowest_scan_point", *iterator);
    const auto& sensorXatLowestScan = visibilityCleanupMap_.at("sensor_x_at_lowest_scan", *iterator);
    const auto& sensorYatLowestScan = visibilityCleanupMap_.at("sensor_y_at_lowest_scan", *iterator);
    const auto& sensorZatLowestScan = visibilityCleanupMap_.at("sensor_z_at_lowest_scan", *iterator);
    if (std::isnan(lowestScanPoint)) {
      continue;
    }
    grid_map::Index indexAtSensor;
    if (!visibilityCleanupMap_.getIndex(grid_map::Position(sensorXatLowestScan, sensorYatLowestScan), indexAtSensor)) {
      continue;
    }
    grid_map::Position point;
    visibilityCleanupMap_.getPosition(*iterator, point);
    float pointDiffX = point.x() - sensorXatLowestScan;
    float pointDiffY = point.y() - sensorYatLowestScan;
    float distanceToPoint = sqrt(pointDiffX * pointDiffX + pointDiffY * pointDiffY);
    if (distanceToPoint > 0.0) {
      for (grid_map::LineIterator lineIt(visibilityCleanupMap_, indexAtSensor, *iterator); !lineIt.isPastEnd(); ++lineIt) {
        grid_map::Position cellPosition;
        visibilityCleanupMap_.getPosition(*lineIt, cellPosition);
        const float cellDiffX = cellPosition.x() - sensorXatLowestScan;
        const float cellDiffY = cellPosition.y() - sensorYatLowestScan;
        const float distanceToCell = distanceToPoint - sqrt(cellDiffX * cellDiffX + cellDiffY * cellDiffY);
        const float maxHeightPoint = lowestScanPoint + (sensorZatLowestScan - lowestScanPoint) / distanceToPoint * distanceToCell;
        auto& cellMaxHeight = visibilityCleanupMap_.at("max_height", *lineIt);
        if (std::isnan(cellMaxHeight) || cellMaxHeight > maxHeightPoint) {
          cellMaxHeight = maxHeightPoint;
        }
      }
    }
  }

  std::vector<grid_map::Position> cellPositionsToRemove;
  for (grid_map::GridMapIterator iterator(visibilityCleanupMap_); !iterator.isPastEnd(); ++iterator) {
    if (!visibilityCleanupMap_.isValid(*iterator)) {
      continue;
    }
    const auto& time = visibilityCleanupMap_.at("time", *iterator);
    if (timeSinceInitialization - time > parameters.scanningDuration_) {
      const auto& elevation = visibilityCleanupMap_.at("elevation", *iterator);
      const auto& variance = visibilityCleanupMap_.at("variance", *iterator);
      const auto& maxHeight = visibilityCleanupMap_.at("max_height", *iterator);
      if (!std::isnan(maxHeight) && elevation - 3.0 * sqrt(variance) > maxHeight) {
        grid_map::Position position;
        visibilityCleanupMap_.getPosition(*iterator, position);
        cellPositionsToRemove.push_back(position);
      }
    }
  }

  scopedLockForRawData.lock();
  for (const auto& cellPosition : cellPositionsToRemove) {
    grid_map::Index index;
    if (!rawMap_.getIndex(cellPosition, index)) {
      continue;
    }
    if (rawMap_.isValid(index)) {
      rawMap_.at("elevation", index) = NAN;
      rawMap_.at("dynamic_time", index) = 0.0f;
    }
  }
  scopedLockForRawData.unlock();

  publishVisibilityCleanupMap();

  const auto duration = std::chrono::steady_clock::now() - methodStartTime;
  RCLCPP_DEBUG(node_->get_logger(), "Visibility cleanup has been performed in %f s (%d points).",
               std::chrono::duration<double>(duration).count(), (int)cellPositionsToRemove.size());
  if (std::chrono::duration<double>(duration).count() > parameters.visibilityCleanupDuration_) {
    RCLCPP_WARN(node_->get_logger(), "Visibility cleanup duration is too high (current rate is %f).",
                1.0 / std::chrono::duration<double>(duration).count());
  }
}

void ElevationMap::move(const Eigen::Vector2d& position) {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  std::vector<grid_map::BufferRegion> newRegions;

  if (rawMap_.move(position, newRegions)) {
    RCLCPP_DEBUG(node_->get_logger(), "Elevation map has been moved to position (%f, %f).", rawMap_.getPosition().x(),
                 rawMap_.getPosition().y());

    grid_map::Matrix& dynTime{rawMap_.get("dynamic_time")};
    dynTime = dynTime.array().isNaN().select(grid_map::Matrix::Scalar(0.0f), dynTime.array());

    if (hasUnderlyingMap_) {
      rawMap_.addDataFrom(underlyingMap_, false, false, true);
    }
  }
}

bool ElevationMap::postprocessAndPublishRawElevationMap() {
  if (!hasRawMapSubscribers()) {
    return false;
  }
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
  grid_map::GridMap rawMapCopy = rawMap_;
  scopedLock.unlock();
  return postprocessorPool_.runTask(rawMapCopy);
}

bool ElevationMap::publishFusedElevationMap() {
  if (!hasFusedMapSubscribers()) {
    return false;
  }
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  grid_map::GridMap fusedMapCopy = fusedMap_;
  scopedLock.unlock();
  fusedMapCopy.add("uncertainty_range", fusedMapCopy.get("upper_bound") - fusedMapCopy.get("lower_bound"));
  grid_map_msgs::msg::GridMap message;
  grid_map::GridMapRosConverter::toMessage(fusedMapCopy, message);
  elevationMapFusedPublisher_->publish(message);
  RCLCPP_DEBUG(node_->get_logger(), "Elevation map (fused) has been published.");
  return true;
}

bool ElevationMap::publishVisibilityCleanupMap() {
  if (visibilityCleanupMapPublisher_->get_subscription_count() < 1) {
    return false;
  }
  boost::recursive_mutex::scoped_lock scopedLock(visibilityCleanupMapMutex_);
  grid_map::GridMap visibilityCleanupMapCopy = visibilityCleanupMap_;
  scopedLock.unlock();
  visibilityCleanupMapCopy.erase("elevation");
  visibilityCleanupMapCopy.erase("variance");
  visibilityCleanupMapCopy.erase("horizontal_variance_x");
  visibilityCleanupMapCopy.erase("horizontal_variance_y");
  visibilityCleanupMapCopy.erase("horizontal_variance_xy");
  visibilityCleanupMapCopy.erase("color");
  visibilityCleanupMapCopy.erase("time");
  grid_map_msgs::msg::GridMap message;
  grid_map::GridMapRosConverter::toMessage(visibilityCleanupMapCopy, message);
  visibilityCleanupMapPublisher_->publish(message);
  RCLCPP_DEBUG(node_->get_logger(), "Visibility cleanup map has been published.");
  return true;
}

grid_map::GridMap& ElevationMap::getRawGridMap() {
  return rawMap_;
}

void ElevationMap::setRawGridMap(const grid_map::GridMap& map) {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_ = map;
}

grid_map::GridMap& ElevationMap::getFusedGridMap() {
  return fusedMap_;
}

void ElevationMap::setFusedGridMap(const grid_map::GridMap& map) {
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_ = map;
}

rclcpp::Time ElevationMap::getTimeOfLastUpdate() {
  return rclcpp::Time(static_cast<int64_t>(rawMap_.getTimestamp()), RCL_ROS_TIME);
}

rclcpp::Time ElevationMap::getTimeOfLastFusion() {
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return rclcpp::Time(static_cast<int64_t>(fusedMap_.getTimestamp()), RCL_ROS_TIME);
}

const kindr::HomTransformQuatD& ElevationMap::getPose() {
  return pose_;
}

bool ElevationMap::getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position) {
  kindr::Position3D positionInGridFrame;
  if (!rawMap_.getPosition3("elevation", index, positionInGridFrame.vector())) {
    return false;
  }
  position = pose_.transform(positionInGridFrame);
  return true;
}

boost::recursive_mutex& ElevationMap::getFusedDataMutex() {
  return fusedMapMutex_;
}

boost::recursive_mutex& ElevationMap::getRawDataMutex() {
  return rawMapMutex_;
}

bool ElevationMap::clean() {
  const Parameters parameters{parameters_.getData()};
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_.get("variance") =
      rawMap_.get("variance").unaryExpr(VarianceClampOperator<float>(parameters.minVariance_, parameters.maxVariance_));
  rawMap_.get("horizontal_variance_x") =
      rawMap_.get("horizontal_variance_x")
          .unaryExpr(VarianceClampOperator<float>(parameters.minHorizontalVariance_, parameters.maxHorizontalVariance_));
  rawMap_.get("horizontal_variance_y") =
      rawMap_.get("horizontal_variance_y")
          .unaryExpr(VarianceClampOperator<float>(parameters.minHorizontalVariance_, parameters.maxHorizontalVariance_));
  return true;
}

void ElevationMap::resetFusedData() {
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_.clearAll();
  fusedMap_.resetTimestamp();
}

void ElevationMap::setFrameId(const std::string& frameId) {
  rawMap_.setFrameId(frameId);
  fusedMap_.setFrameId(frameId);
}

void ElevationMap::setTimestamp(rclcpp::Time timestamp) {
  rawMap_.setTimestamp(static_cast<uint64_t>(timestamp.nanoseconds()));
  fusedMap_.setTimestamp(static_cast<uint64_t>(timestamp.nanoseconds()));
}

const std::string& ElevationMap::getFrameId() {
  return rawMap_.getFrameId();
}

bool ElevationMap::hasRawMapSubscribers() const {
  return postprocessorPool_.pipelineHasSubscribers();
}

bool ElevationMap::hasFusedMapSubscribers() const {
  return elevationMapFusedPublisher_->get_subscription_count() >= 1;
}

void ElevationMap::underlyingMapCallback(const grid_map_msgs::msg::GridMap::ConstSharedPtr& underlyingMap) {
  const Parameters parameters{parameters_.getData()};
  RCLCPP_INFO(node_->get_logger(), "Updating underlying map.");
  grid_map::GridMapRosConverter::fromMessage(*underlyingMap, underlyingMap_);
  if (underlyingMap_.getFrameId() != rawMap_.getFrameId()) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "The underlying map does not have the same map frame ('"
                                                 << underlyingMap_.getFrameId() << "') as the elevation map ('" << rawMap_.getFrameId()
                                                 << "').");
    return;
  }
  if (!underlyingMap_.exists("elevation")) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "The underlying map does not have an 'elevation' layer.");
    return;
  }
  if (!underlyingMap_.exists("variance")) {
    underlyingMap_.add("variance", parameters.minVariance_);
  }
  if (!underlyingMap_.exists("horizontal_variance_x")) {
    underlyingMap_.add("horizontal_variance_x", parameters.minHorizontalVariance_);
  }
  if (!underlyingMap_.exists("horizontal_variance_y")) {
    underlyingMap_.add("horizontal_variance_y", parameters.minHorizontalVariance_);
  }
  if (!underlyingMap_.exists("color")) {
    underlyingMap_.add("color", 0.0);
  }
  underlyingMap_.setBasicLayers(rawMap_.getBasicLayers());
  hasUnderlyingMap_ = true;
  rawMap_.addDataFrom(underlyingMap_, false, false, true);
}

void ElevationMap::setRawSubmapHeight(const grid_map::Position& initPosition, float mapHeight, float variance, double lengthInXSubmap,
                                      double lengthInYSubmap) {
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);

  const grid_map::Position topLeftPosition(initPosition(0) + lengthInXSubmap / 2, initPosition(1) + lengthInYSubmap / 2);
  grid_map::Index submapTopLeftIndex;
  rawMap_.getIndex(topLeftPosition, submapTopLeftIndex);

  const double resolution = rawMap_.getResolution();
  const int lengthInXSubmapI = static_cast<int>(lengthInXSubmap / resolution);
  const int lengthInYSubmapI = static_cast<int>(lengthInYSubmap / resolution);
  const Eigen::Array2i submapBufferSize(lengthInYSubmapI, lengthInXSubmapI);

  grid_map::Matrix& elevationData = rawMap_["elevation"];
  grid_map::Matrix& varianceData = rawMap_["variance"];
  for (grid_map::SubmapIterator iterator(rawMap_, submapTopLeftIndex, submapBufferSize); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    elevationData(index(0), index(1)) = mapHeight;
    varianceData(index(0), index(1)) = variance;
  }
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation) {
  return 0.5 * erfc(-(x - mean) / (standardDeviation * sqrt(2.0)));
}

}  // namespace elevation_mapping
