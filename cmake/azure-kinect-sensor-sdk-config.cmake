# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

if (azure-kinect-sdk_CONFIG_INCLUDED)
  return()
endif()
set(azure-kinect-sdk_CONFIG_INCLUDED TRUE)

set(azure-kinect-sdk_INCLUDE_DIRS "C:/Program Files/Azure Kinect SDK/sdk/include")

foreach(lib k4a;k4arecord)

  set(onelib "${lib}-NOTFOUND")
  find_library(onelib ${lib}
      PATHS "C:/Program Files/Azure Kinect SDK/sdk/windows-desktop/amd64/release/lib"
      NO_DEFAULT_PATH)

  if((NOT onelib) OR (onelib EQUAL "${lib}-NOTFOUND"))
    message(FATAL_ERROR "Library '${lib}' in package azure-kinect-sdk is not installed properly")
  endif()

  set(onebin "${lib}-NOTFOUND")
  find_file(onebin "${lib}.dll"
      PATHS "C:/Program Files/Azure Kinect SDK/sdk/windows-desktop/amd64/release/bin"
      NO_DEFAULT_PATH)

  if((NOT onebin) OR (onebin EQUAL "${lib}-NOTFOUND"))
    message(FATAL_ERROR "Binary '${lib}.dll' in package azure-kinect-sdk is not installed properly")
  endif()

  add_library(azure-kinect-sensor-sdk-${lib} SHARED IMPORTED GLOBAL)
  set_property(TARGET azure-kinect-sensor-sdk-${lib} PROPERTY IMPORTED_LOCATION ${onebin})
  set_property(TARGET azure-kinect-sensor-sdk-${lib} PROPERTY IMPORTED_IMPLIB ${onelib})

  list(APPEND azure-kinect-sdk_LIBRARIES azure-kinect-sensor-sdk-${lib})

endforeach()

foreach(bin libusb-1.0;depthengine_1_0)

  set(onebin "${bin}-NOTFOUND")
  find_file(onebin "${bin}.dll"
      PATHS "C:/Program Files/Azure Kinect SDK/sdk/windows-desktop/amd64/release/bin"
      NO_DEFAULT_PATH)

  if((NOT onebin) OR (onebin EQUAL "${lib}-NOTFOUND"))
    message(FATAL_ERROR "Binary '${bin}.dll' in package azure-kinect-sdk is not installed properly")
  endif()

  add_library(azure-kinect-sensor-sdk-${bin} SHARED IMPORTED GLOBAL)
  set_property(TARGET azure-kinect-sensor-sdk-${bin} PROPERTY IMPORTED_LOCATION ${onebin})

  list(APPEND azure-kinect-sdk_BINARIES azure-kinect-sensor-sdk-${bin})

endforeach()
