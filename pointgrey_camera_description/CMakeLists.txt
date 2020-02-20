cmake_minimum_required(VERSION 2.8.3)
project(pointgrey_camera_description)

find_program(XCACHE_FOUND xcache_client)
if (XCACHE_FOUND)
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE xcache_client)
endif(XCACHE_FOUND)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
