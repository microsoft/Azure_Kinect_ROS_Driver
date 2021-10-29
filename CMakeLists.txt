# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(azure_kinect_ros_driver LANGUAGES C CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  nodelet
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nodelet
  cv_bridge
  camera_info_manager
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES k4a_ros_bridge
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/k4a_ros_bridge_node.cpp
  src/k4a_ros_device.cpp
  src/k4a_ros_device_params.cpp
  src/k4a_calibration_transform_data.cpp
  )

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_11)

add_library(${PROJECT_NAME}_nodelet
  src/k4a_ros_bridge_nodelet.cpp
  src/k4a_ros_device.cpp
  src/k4a_ros_device_params.cpp
  src/k4a_calibration_transform_data.cpp
  )

target_compile_features(${PROJECT_NAME}_nodelet PUBLIC cxx_std_11)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

############################
#### AZURE KINECT SDK ######
############################

message("Finding K4A SDK binaries")

# Disable cached locations for K4A SDK binaries.
# Do this to force the search logic to happen correctly.
# If we don't disable these cached directories, we
# won't be able to tell the difference between the ext/sdk location
# and the system installed version on linux. Since we have to treat these
# differently (one needs install, one doesn't) we must disable the cache
# so that find_package(k4a) will fail in all cases if not installed via the .deb.
unset(k4a_DIR CACHE)

# Force running the Findk4a.cmake module
find_package(k4a 1.3.0 QUIET MODULE REQUIRED)
set(K4A_LIBS k4a::k4a;k4a::k4arecord)

# Try to find and enable the body tracking SDK
find_package(k4abt 1.0.0 QUIET MODULE)
if (k4abt_FOUND)
  list(APPEND K4A_LIBS k4abt::k4abt)
  message(STATUS "Body Tracking SDK found: compiling support for Body Tracking")
  target_compile_definitions(${PROJECT_NAME}_node PUBLIC K4A_BODY_TRACKING)
  target_compile_definitions(${PROJECT_NAME}_nodelet PUBLIC K4A_BODY_TRACKING)
else()
  message("!!! Body Tracking SDK not found: body tracking features will not be available !!!")
endif()

# This reads the K4A_LIBS and K4A_INSTALL_REQUIRED variables and decides how to install
# the various shared objects / DLLs
include(Installk4a)

##################################
###### END AZURE KINECT SDK ######
##################################

include_directories(
  ${catkin_INCLUDE_DIRS}
  "include"
)

target_link_libraries(${PROJECT_NAME}_node
  ${K4A_LIBS}
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${K4A_LIBS}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
