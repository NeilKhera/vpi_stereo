cmake_minimum_required(VERSION 3.0.2)
project(vpi_stereo)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_library(vpi_utils_library include/vpi_utils/src/Disparity.cpp)

add_subdirectory(include/vpi_utils)

add_executable(vpi_stereo_node src/vpi_stereo_node.cpp)
target_link_libraries(vpi_stereo_node ${catkin_LIBRARIES} vpi_utils_library)
