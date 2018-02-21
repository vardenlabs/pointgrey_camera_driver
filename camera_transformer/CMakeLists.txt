cmake_minimum_required(VERSION 2.8.3)
project(camera_transformer)

###########
## Build ##
###########
# set compiler flags
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_C_FLAGS_RELEASE "-O2")
add_definitions(-std=c++11)

# add modules
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules")
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  image_transport
  compressed_image_transport
  cv_bridge
  std_msgs
  perception_msgs
  sensor_msgs
  dynamic_reconfigure
)
find_package(Eigen REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(resource_monitor REQUIRED)

catkin_package(
   CATKIN_DEPENDS roscpp roslib
)

#SET(CMAKE_CXX_FLAGS_DEBUG "-O0")
#SET(CMAKE_C_FLAGS_DEBUG "-O0")

set(PROJECT_INCLUDE_DIRS ${resource_monitor_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

set(PROJECT_LIBRARIES ${resource_monitor_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES})

include_directories(${PROJECT_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME})

add_executable(camera_transformer_node src/camera_transformer_node.cpp)
target_link_libraries(camera_transformer_node ${PROJECT_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
