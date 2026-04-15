cmake_minimum_required(VERSION 3.5)
project(elevation_mapping)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(filters REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

###########
# Library #
###########

# Our custom pcl type with precompiled template instantiations.
add_library(${PROJECT_NAME}_pcl_types
  src/PointXYZRGBConfidenceRatio.cpp
)
ament_target_dependencies(${PROJECT_NAME}_pcl_types
  rclcpp
  pcl_conversions
  sensor_msgs
  grid_map_core
)
target_include_directories(${PROJECT_NAME}_pcl_types PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_options(${PROJECT_NAME}_pcl_types PRIVATE ${PCL_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_pcl_types
  ${PCL_LIBRARIES}
)

add_library(${PROJECT_NAME}_library
  src/ElevationMapping.cpp
  src/ElevationMap.cpp
  src/input_sources/Input.cpp
  src/input_sources/InputSourceManager.cpp
  src/postprocessing/PostprocessorPool.cpp
  src/postprocessing/PostprocessingWorker.cpp
  src/postprocessing/PostprocessingPipelineFunctor.cpp
  src/RobotMotionMapUpdater.cpp
  src/sensor_processors/SensorProcessorBase.cpp
  src/sensor_processors/StructuredLightSensorProcessor.cpp
  src/sensor_processors/StereoSensorProcessor.cpp
  src/sensor_processors/LaserSensorProcessor.cpp
  src/sensor_processors/PerfectSensorProcessor.cpp
)

target_include_directories(${PROJECT_NAME}_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_library PRIVATE ${PCL_CXX_FLAGS})

ament_target_dependencies(${PROJECT_NAME}_library
  rclcpp
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  sensor_msgs
  std_srvs
  std_msgs
  geometry_msgs
  message_filters
  grid_map_ros
  grid_map_msgs
  grid_map_core
  filters
)

target_link_libraries(${PROJECT_NAME}_library
  ${PROJECT_NAME}_pcl_types
  pthread
)

##############
# Executable #
##############

add_executable(${PROJECT_NAME}
  src/elevation_mapping_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_pcl_types
    ${PROJECT_NAME}_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_${PROJECT_NAME}_cumulative_distribution
    test/ElevationMapTest.cpp
    test/test_elevation_mapping.cpp
    test/WeightedEmpiricalCumulativeDistributionFunctionTest.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_cumulative_distribution
    ${PROJECT_NAME}_library
  )
  target_include_directories(test_${PROJECT_NAME}_cumulative_distribution
    PRIVATE include
    SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_${PROJECT_NAME}_cumulative_distribution
    rclcpp
    grid_map_ros
    grid_map_core
  )

  ament_add_gtest(test_${PROJECT_NAME}_thread_safe_parameter_wrapper
    test/ThreadSafeParameterWrapperTest.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_thread_safe_parameter_wrapper
    ${PROJECT_NAME}_library
  )
  target_include_directories(test_${PROJECT_NAME}_thread_safe_parameter_wrapper
    PRIVATE include
  )
  ament_target_dependencies(test_${PROJECT_NAME}_thread_safe_parameter_wrapper
    rclcpp
  )
endif()

ament_package()
