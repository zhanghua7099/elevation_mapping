/*
 * PostprocessorPool.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

#include <rclcpp/rclcpp.hpp>

namespace elevation_mapping {

PostprocessorPool::PostprocessorPool(std::size_t poolSize, rclcpp::Node* node) {
  for (std::size_t i = 0; i < poolSize; ++i) {
    workers_.emplace_back(std::make_unique<PostprocessingWorker>(node));
    availableServices_.push_back(i);
  }
}

PostprocessorPool::~PostprocessorPool() {
  for (auto& worker : workers_) {
    worker->ioService().stop();
  }

  for (auto& worker : workers_) {
    try {
      if (worker->thread().joinable()) {
        worker->thread().join();
      }
    } catch (const std::exception&) {
    }
  }
}

bool PostprocessorPool::runTask(const GridMap& gridMap) {
  size_t serviceIndex{0};
  {
    boost::lock_guard<boost::mutex> lock(availableServicesMutex_);
    if (availableServices_.empty()) {
      return false;
    }
    serviceIndex = availableServices_.back();
    availableServices_.pop_back();
  }

  workers_.at(serviceIndex)->setDataBuffer(gridMap);

  auto task = [this, serviceIndex] { wrapTask(serviceIndex); };
  workers_.at(serviceIndex)->ioService().post(task);
  return true;
}

void PostprocessorPool::wrapTask(size_t serviceIndex) {
  try {
    GridMap postprocessedMap = workers_.at(serviceIndex)->processBuffer();
    workers_.at(serviceIndex)->publish(postprocessedMap);
  } catch (const std::exception& exception) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("elevation_mapping"),
                        "Postprocessor pipeline, thread " << serviceIndex << " experienced an error: " << exception.what());
  }

  boost::unique_lock<boost::mutex> lock(availableServicesMutex_);
  availableServices_.push_back(serviceIndex);
}

bool PostprocessorPool::pipelineHasSubscribers() const {
  return std::all_of(workers_.cbegin(), workers_.cend(),
                     [](const std::unique_ptr<PostprocessingWorker>& worker) { return worker->hasSubscribers(); });
}

}  // namespace elevation_mapping
