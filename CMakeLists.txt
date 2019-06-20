# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 2.8.3)
project(azure-kinect-ros-driver LANGUAGES C CXX)


if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  add_compile_options(/std:c++latest)
else()
  add_compile_options(-std=c++11)
endif()

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

add_library(${PROJECT_NAME}_nodelet
  src/k4a_ros_bridge_nodelet.cpp
  src/k4a_ros_device.cpp
  src/k4a_ros_device_params.cpp
  src/k4a_calibration_transform_data.cpp
  )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

############################
#### AZURE KINECT SDK ######
############################

# Define the names of some files that we are going to try and find
if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
  set(DEPTHENGINE_DLL "depthengine_1_0.dll")
  set(LIBUSB_DLL "libusb-1.0.dll")
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(DEPTHENGINE_DLL "libdepthengine.so.1.0")
else()
  message(FATAL_ERROR "Platform ${CMAKE_SYSTEM_NAME} is not supported")
endif()

message("Finding K4A SDK binaries")

# First try to find an SDK in the ext/sdk folder
# Looking in this folder should be platform agnostic
find_package(k4a 1.1.0 QUIET
  PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk")

if (${k4a_FOUND})
  message(STATUS "K4A SDK found in ext/sdk!")
  message(STATUS "!! Not searching for SDK in system path !!")

  # mark that the K4A SDK will need to be copied to the output binary folder
  # this is needed even on Linux if the SDK is not installed to the system path
  set(K4A_INSTALL_NEEDED true)

  get_target_property(K4A_INCLUDE_DIRS k4a::k4a INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(K4A_CONFIGS k4a::k4a IMPORTED_CONFIGURATIONS)
  message("Configs: ${K4A_CONFIGS}")

  # TODO: if we find more than one configuration, we should fail
  # TODO: potentially clean this logic up
  foreach(imported_config ${K4A_CONFIGS})
    if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
      get_target_property(K4A_CONFIG_LIBS k4a::k4a IMPORTED_IMPLIB_${imported_config})
      get_target_property(K4A_CONFIG_DLLS k4a::k4a IMPORTED_LOCATION_${imported_config})
      list(APPEND K4A_LIBS ${K4A_CONFIG_LIBS})
      list(APPEND K4A_DLL_FILES ${K4A_CONFIG_DLLS})
    elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
      get_target_property(K4A_CONFIG_LIBS k4a::k4a IMPORTED_LOCATION_${imported_config})
      get_target_property(K4A_CONFIG_DLLS k4a::k4a IMPORTED_LOCATION_${imported_config})
      list(APPEND K4A_LIBS ${K4A_CONFIG_LIBS})
      list(APPEND K4A_DLL_FILES ${K4A_CONFIG_DLLS})
    endif()
  endforeach()

  if(DEFINED DEPTHENGINE_DLL)
    ## Find the depth engine DLL
    file(GLOB_RECURSE DEPTHENGINE_DLL_FILE
      "${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk/*/${DEPTHENGINE_DLL}" )
    list(APPEND K4A_DLL_FILES ${DEPTHENGINE_DLL_FILE})
  endif()

  if(DEFINED LIBUSB_DLL)
    ## Find LibUSB
    file(GLOB_RECURSE LIBUSB_DLL_FILE
      "${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk/*/${LIBUSB_DLL}" )
    list(APPEND K4A_DLL_FILES ${LIBUSB_DLL_FILE})
  endif()

else()
  
  if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    # Windows will always need K4A install
    set(K4A_INSTALL_NEEDED true)

    find_package(azure-kinect-sensor-sdk 1.1.0 EXACT REQUIRED 
      PATHS "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

    message("K4A SDK found in Program Files!")

    list(APPEND K4A_LIBS ${azure-kinect-sensor-sdk_LIBRARIES})
    list(APPEND K4A_BINS ${azure-kinect-sensor-sdk_BINARIES})
    set(K4A_INCLUDE_DIRS ${azure-kinect-sensor-sdk_INCLUDE_DIRS})

    # Create a list of all K4A DLLs
    list(APPEND K4A_DLLS ${azure-kinect-sensor-sdk_LIBRARIES})
    list(APPEND K4A_DLLS ${azure-kinect-sensor-sdk_BINARIES})

    foreach(DLL ${K4A_DLLS})
      set(prop "PROPERTY-NOTFOUND")
      get_property(prop TARGET ${DLL} PROPERTY IMPORTED_LOCATION)
      message(STATUS "IMPORTED_LOCATION: ${prop}")

      if((NOT prop) OR (prop EQUAL "PROPERTY-NOTFOUND"))
        message(FATAL_ERROR "Target '${DLL}' in package azure-kinect-sensor-sdk does not contain an IMPORTED_LOCATION property")
      endif()

      list(APPEND K4A_DLL_FILES ${prop})
    endforeach(DLL)

  elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    # TODO: linux support for preinstalled packages
    message(FATAL_ERROR "Platform ${CMAKE_SYSTEM_NAME} is not supported for system installs yet. Please extract an SDK and Depth Engine to ext/sdk")
  endif()
endif()

message("K4A Include Dirs: ${K4A_INCLUDE_DIRS}")
message("K4A Libs: ${K4A_LIBS}")
message("K4A DLLs: ${K4A_DLL_FILES}")
message("K4A Install Needed: ${K4A_INSTALL_NEEDED}")

if (${K4A_INSTALL_NEEDED})
  ## Tell cmake that we need to reconfigure if any of the DLL files change
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${K4A_DLL_FILES})

  ## We need to copy the DLLs into the CATKIN_PACKAGE_LIB_DESTINATION so
  ## the node executable can find them on launch
  if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(DLL_COPY_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}")
  elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(DLL_COPY_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")
  endif()

  file(MAKE_DIRECTORY "${DLL_COPY_DIRECTORY}")

  # TODO: this code only runs on initial compilation. If the underlying SDK is updated,
  # the new binaries aren't copied into the ROS devel directory. Somehow, cmake needs to be made
  # aware that these files are dependencies to be tracked.

  foreach(DLL ${K4A_DLL_FILES})
    file(COPY "${DLL}" DESTINATION "${DLL_COPY_DIRECTORY}")
    message(STATUS "Copied dll from ${DLL} to ${DLL_COPY_DIRECTORY}")
  endforeach(DLL)
endif()

##################################
###### END AZURE KINECT SDK ######
##################################

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${K4A_INCLUDE_DIRS}
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
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  DIRECTORY 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Run a custom install script for the K4A components
## Running the two "CODE" blocks populates the cmake_install.cmake script with the information
## about which DLLs to install, and where to install them.
## We then run the more complex script to actually perform the installation.
if (${K4A_INSTALL_NEEDED})
  message("Installing K4A SDK to binary output folder")
  install(CODE "set(K4A_DLL_FILES \"${K4A_DLL_FILES}\")")
  install(CODE "set(DLL_COPY_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}\")")
  install(SCRIPT "./cmake/azure-kinect-sensor-sdk-install.cmake")
endif()
