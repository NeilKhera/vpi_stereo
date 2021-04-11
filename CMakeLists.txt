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
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vpi_stereo_node src/vpi_stereo_node.cpp)
target_link_libraries(vpi_stereo_node ${catkin_LIBRARIES})
