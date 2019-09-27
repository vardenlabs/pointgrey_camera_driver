cmake_minimum_required(VERSION 2.8.3)
project(wfov_camera_msgs)

find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif(CCACHE_FOUND)

find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  WFOVCompressedImage.msg
  WFOVImage.msg
  WFOVTrigger.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
