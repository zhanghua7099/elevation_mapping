/*
 * PostprocessorPool.hpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <grid_map_core/GridMap.hpp>
#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "elevation_mapping/postprocessing/PostprocessingWorker.hpp"

namespace elevation_mapping {

class PostprocessorPool {
 public:
  using GridMap = grid_map::GridMap;

  PostprocessorPool(std::size_t poolSize, rclcpp::Node* node);

  ~PostprocessorPool();

  bool runTask(const GridMap& gridMap);

  bool pipelineHasSubscribers() const;

 private:
  void wrapTask(size_t serviceIndex);

  std::vector<std::unique_ptr<PostprocessingWorker>> workers_;

  boost::mutex availableServicesMutex_;
  std::deque<size_t> availableServices_;
};

}  // namespace elevation_mapping
