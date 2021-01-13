# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(azure_kinect_ros_driver LANGUAGES C CXX)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

option(CUDA_SUPPORT "use CUDA support onnxruntime library" ON)

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
if(MSVC)
  if(CUDA_SUPPORT)
      set(KINECT_DEPENDENCIES "Microsoft.Azure.Kinect.BodyTracking.Dependencies.0.9.1")
      set(KINECT_URL "https://www.nuget.org/api/v2/package/Microsoft.Azure.Kinect.BodyTracking.Dependencies/0.9.1")
      set(KINECT_SHA512 "5df5ceb2f7905a3e208a085a29ef02feb1820ffe819563af77c272ad7e068cf3a158c0ce610e421829b5f7ebbb628c45f56617344d6d1ef0a31d719253cf881f")
      file(DOWNLOAD
      ${KINECT_URL}
      ${CMAKE_CURRENT_BINARY_DIR}/kinect.nuget
      EXPECTED_HASH SHA512=${KINECT_SHA512}
      SHOW_PROGRESS
      )

      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${KINECT_DEPENDENCIES}")
        execute_process(COMMAND tar xvzf "${CMAKE_CURRENT_BINARY_DIR}/kinect.nuget" 
                    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${KINECT_DEPENDENCIES}"
      )
        
      message(STATUS "${CATKIN_PACKAGE_SHARE_DESTINATION} the share destination")

      file(GLOB KINECT_DLLS
        "${CMAKE_CURRENT_BINARY_DIR}/${KINECT_DEPENDENCIES}/lib/native/amd64/release/*.dll"
      )

      message(STATUS "kinect dlls ${KINECT_DLLS}")
      file(COPY ${KINECT_DLLS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
      file(COPY ${KINECT_DLLS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

      set(CUDNN_DEPENDENCIES "Microsoft.Azure.Kinect.BodyTracking.Dependencies.cuDNN.0.9.1")
      set(CUDNN_URL "https://www.nuget.org/api/v2/package/Microsoft.Azure.Kinect.BodyTracking.Dependencies.cuDNN/0.9.1")
      set(CUDNN_SHA512 "a9986d5d7f06e0445cb65dfe1363c3e827df2c87b80ecfa05894c062714bf2092b3f87defd136cd2a89b9e588e773080ecf7079e8ce7e428426c4a00259c5085")
      file(DOWNLOAD
      ${CUDNN_URL}
      ${CMAKE_CURRENT_BINARY_DIR}/cudnn.nuget
      EXPECTED_HASH SHA512=${CUDNN_SHA512}
      SHOW_PROGRESS
      )

      file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CUDNN_DEPENDENCIES}")

      execute_process(COMMAND tar xvzf "${CMAKE_CURRENT_BINARY_DIR}/cudnn.nuget" 
                  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CUDNN_DEPENDENCIES}"
                  )
      
      file(GLOB CUDNN_DLLS
      "${CMAKE_CURRENT_BINARY_DIR}/${CUDNN_DEPENDENCIES}/lib/native/amd64/release/*.dll"
      )
      file(COPY ${CUDNN_DLLS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
      file(COPY ${CUDNN_DLLS} DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION})

  else()
    message(FATAL_ERROR "Body Tracking SDK needs a NVIDIA GEFORCE GTX 1070 or better")
  endif()
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
