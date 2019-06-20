# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

if (azure-kinect-sensor-sdk_CONFIG_INCLUDED)
  return()
endif()
set(azure-kinect-sensor-sdk_CONFIG_INCLUDED TRUE)

set(K4A_INSTALL_PATH "C:/Program Files/Azure Kinect SDK v${azure-kinect-sensor-sdk_FIND_VERSION}")
set(K4A_LIB_PATH "${K4A_INSTALL_PATH}/sdk/windows-desktop/amd64/release/lib")
set(K4A_BIN_PATH "${K4A_INSTALL_PATH}/sdk/windows-desktop/amd64/release/bin")

set(azure-kinect-sensor-sdk_INCLUDE_DIRS "${K4A_INSTALL_PATH}/sdk/include")

foreach(lib k4a;k4arecord)

  set(onelib "${lib}-NOTFOUND")
  find_library(onelib ${lib}
      PATHS "${K4A_LIB_PATH}"
      NO_DEFAULT_PATH)

  if((NOT onelib) OR (onelib EQUAL "${lib}-NOTFOUND"))
    message(WARNING "Library '${lib}' not found in ${K4A_LIB_PATH}")
    message(FATAL_ERROR "Library '${lib}' in package azure-kinect-sensor-sdk is not installed properly")
  endif()

  set(onebin "${lib}-NOTFOUND")
  find_file(onebin "${lib}.dll"
      PATHS "${K4A_BIN_PATH}"
      NO_DEFAULT_PATH)

  if((NOT onebin) OR (onebin EQUAL "${lib}-NOTFOUND"))
    message(WARNING "Binary '${lib}' not found in ${K4A_BIN_PATH}")
    message(FATAL_ERROR "Binary '${lib}.dll' in package azure-kinect-sensor-sdk is not installed properly")
  endif()

  add_library(azure-kinect-sensor-sdk-${lib} SHARED IMPORTED GLOBAL)
  set_property(TARGET azure-kinect-sensor-sdk-${lib} PROPERTY IMPORTED_LOCATION ${onebin})
  set_property(TARGET azure-kinect-sensor-sdk-${lib} PROPERTY IMPORTED_IMPLIB ${onelib})

  list(APPEND azure-kinect-sensor-sdk_LIBRARIES azure-kinect-sensor-sdk-${lib})

endforeach()

foreach(bin depthengine_1_0)

  set(onebin "${bin}-NOTFOUND")
  find_file(onebin "${bin}.dll"
      PATHS "${K4A_BIN_PATH}"
      NO_DEFAULT_PATH)

  if((NOT onebin) OR (onebin EQUAL "${bin}-NOTFOUND"))
    message(WARNING "Binary '${bin}' not found in ${K4A_BIN_PATH}")
    message(FATAL_ERROR "Binary '${bin}.dll' in package azure-kinect-sensor-sdk is not installed properly")
  endif()

  add_library(azure-kinect-sensor-sdk-${bin} SHARED IMPORTED GLOBAL)
  set_property(TARGET azure-kinect-sensor-sdk-${bin} PROPERTY IMPORTED_LOCATION ${onebin})

  list(APPEND azure-kinect-sensor-sdk_BINARIES azure-kinect-sensor-sdk-${bin})

endforeach()
