cmake_minimum_required(VERSION 3.5)
project(elevation_mapping_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config doc launch rviz sample_data
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/tf_to_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
