cmake_minimum_required(VERSION 3.5)
project(elevation_mapping_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config doc launch sample_data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
