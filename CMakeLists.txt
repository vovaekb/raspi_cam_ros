cmake_minimum_required(VERSION 2.8.3)
project(raspi_cam_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
)


link_directories(/usr/lib/uv4l/uv4lext/armv6l/)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES raspi_cam_ros
#  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(capture src/capturer.cpp)
target_link_libraries(capture ${catkin_LIBRARIES} uv4lext)
