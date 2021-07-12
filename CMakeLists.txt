# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(azure_kinect_ros_driver LANGUAGES C CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_options(/wd4251)
  # WORKAROUND: warning C4251: 'cv_bridge::CvImage::tracked_object_': class 'std::shared_ptr<const void>' needs to have dll-interface to be used by clients of class 'cv_bridge::CvImage'
endif()

## Find packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

###########
## Build ##
###########


## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
  src/k4a_ros_bridge_node.cpp
  src/k4a_ros_device.cpp
  src/k4a_ros_device_params.cpp
  src/k4a_calibration_transform_data.cpp
  )

ament_target_dependencies(${PROJECT_NAME}_node rclcpp 
  std_msgs 
  sensor_msgs
  visualization_msgs
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge)

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
else()
  message("!!! Body Tracking SDK not found: body tracking features will not be available !!!")
endif()

# This reads the K4A_LIBS and K4A_INSTALL_REQUIRED variables and decides how to install
# the various shared objects / DLLs
include(Installk4a)

##################################
###### END AZURE KINECT SDK ######
##################################
include_directories(include)

target_link_libraries(${PROJECT_NAME}_node
  ${K4A_LIBS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()