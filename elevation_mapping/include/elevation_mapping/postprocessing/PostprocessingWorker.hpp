/*
 * PostprocessingWorker.hpp
 *
 *  Created on: Dec. 21, 2020
 *      Author: Yoshua Nava
 *   Institute: ANYbotics
 */

#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <thread>

#include <grid_map_core/GridMap.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

class PostprocessingWorker {
 public:
  using GridMap = grid_map::GridMap;

  explicit PostprocessingWorker(rclcpp::Node* node);

  boost::asio::io_service& ioService() { return ioService_; }
  std::thread& thread() { return thread_; }
  const GridMap& dataBuffer() { return dataBuffer_; }
  void setDataBuffer(GridMap data) { dataBuffer_ = std::move(data); }

  GridMap processBuffer();
  void publish(const GridMap& gridMap) const;
  bool hasSubscribers() const;

 protected:
  PostprocessingPipelineFunctor functor_;
  boost::asio::io_service ioService_;
  boost::asio::io_service::work work_;
  std::thread thread_;
  GridMap dataBuffer_;
};

}  // namespace elevation_mapping
