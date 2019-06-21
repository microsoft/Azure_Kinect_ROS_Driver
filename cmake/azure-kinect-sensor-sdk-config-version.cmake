# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

set(versionfile "NOTFOUND")
find_file(versionfile "version.txt"
      PATHS "C:/Program Files/Azure Kinect SDK v${PACKAGE_FIND_VERSION}"
      NO_DEFAULT_PATH)

if(NOT versionfile)
  message(STATUS "Azure Kinect SDK is not installed to Program Files.")
  return()
endif()

set(versionfilecontents "0.0.0")
file(READ ${versionfile} versionfilecontents)

message(STATUS "Azure Kinect SDK Version File Contents: ${versionfilecontents}")

## Version file contents should be in the format X.X.X-BETA (with the -BETA being optional)
set(version "0.0.0")
if(${versionfilecontents} MATCHES "^([0-9]+)[.]([0-9]+)[.]([0-9]+)(-.+)?")

  set(PACKAGE_VERSION "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}")

  message(STATUS "PACKAGE_VERSION: ${PACKAGE_VERSION}")
  message(STATUS "PACKAGE_FIND_VERSION: ${PACKAGE_FIND_VERSION}")

  if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
  else()

    if(PACKAGE_FIND_VERSION_MAJOR STREQUAL ${CMAKE_MATCH_1})
      set(PACKAGE_VERSION_COMPATIBLE TRUE)
    else()
      set(PACKAGE_VERSION_COMPATIBLE FALSE)
    endif()

    if(PACKAGE_FIND_VERSION STREQUAL PACKAGE_VERSION)
        set(PACKAGE_VERSION_EXACT TRUE)
    endif()
  endif()
else()
  message(STATUS "Regex failed")
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
endif()


# if the installed or the using project don't have CMAKE_SIZEOF_VOID_P set, ignore it:
if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "" OR "8" STREQUAL "")
   return()
endif()

# check that the installed version has the same 32/64bit-ness as the one which is currently searching:
if(NOT CMAKE_SIZEOF_VOID_P STREQUAL "8")
  math(EXPR installedBits "8 * 8")
  set(PACKAGE_VERSION "${PACKAGE_VERSION} (${installedBits}bit)")
  set(PACKAGE_VERSION_UNSUITABLE TRUE)
endif()
