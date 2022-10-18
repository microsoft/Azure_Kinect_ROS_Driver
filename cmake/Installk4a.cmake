
foreach(_lib ${K4A_LIBS})
    # Sometimes the k4a package exposes IMPORTED_LOCATION_RELWITHDEBINFO, sometimes it just exposes IMPORTED_LOCATION
    # This section handles both possible cases
    set(_configs "PROPERTY-NOTFOUND")
    get_target_property(_configs ${_lib} IMPORTED_CONFIGURATIONS)

    ## TODO: I tried using CMake's list(LENGTH) on the output of IMPORTED_CONFIGURATIONS so that
    ## I could assert that there was only one imported configuration, but for some reason the list length
    ## always returned 0. Potentially revisit this check in future.

    set(_dll "PROPERTY-NOTFOUND")

    if (_configs)
        get_target_property(_dll ${_lib} IMPORTED_LOCATION_${_configs})
    else()
        get_target_property(_dll ${_lib} IMPORTED_LOCATION)
    endif()

    if((NOT _dll) OR (_dll EQUAL "PROPERTY-NOTFOUND"))
        message(FATAL_ERROR "Target '${_lib}' in package k4a does not contain an IMPORTED_LOCATION** property")
    endif()

    if(NOT EXISTS ${_dll})
        message(FATAL_ERROR "Target '${_lib}' in package k4a requires ${_dll}, which was not found")
    endif()

    list(APPEND K4A_DLL_FILES ${_dll})

    # Check and see if any of these targets depend on other DLLs (for example, depthengine or libusb)
    set(_dependent_dlls "PROPERTY-NOTFOUND")
    get_property(_dependent_dlls TARGET ${_lib} PROPERTY IMPORTED_LINK_DEPENDENT_LIBRARIES)

    if(_dependent_dlls AND (NOT (_dependent_dlls EQUAL "PROPERTY-NOTFOUND")))
        foreach(_dependent_dll ${_dependent_dlls})
            if(NOT EXISTS ${_dependent_dll})
                message(FATAL_ERROR "Target '${_lib}' in package k4a requires ${_dependent_dll}, which was not found")
            endif()
            list(APPEND K4A_DLL_FILES ${_dependent_dll})
        endforeach()
    endif()
endforeach()

message(STATUS "K4A Libs: ${K4A_LIBS}")
message(STATUS "K4A DLLs: ${K4A_DLL_FILES}")
message(STATUS "K4A Install Needed: ${K4A_INSTALL_NEEDED}")

if (${K4A_INSTALL_NEEDED})
  # Tell cmake that we need to reconfigure if any of the DLL files change
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${K4A_DLL_FILES})

  # We need to copy the DLLs into the CATKIN_PACKAGE_LIB_DESTINATION so
  # the node executable can find them on launch, and CATKIN_PACKAGE_BIN_DESTINATION
  # so the nodelet can find them on launch
  set(DLL_COPY_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION};${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")

  foreach(DIRECTORY ${DLL_COPY_DIRECTORY})
    file(MAKE_DIRECTORY "${DIRECTORY}")
  endforeach(DIRECTORY)

  foreach(DLL ${K4A_DLL_FILES})
    foreach(DIRECTORY ${DLL_COPY_DIRECTORY})
      file(COPY "${DLL}" DESTINATION "${DIRECTORY}")
      get_filename_component(DLL_NAME ${DLL} NAME)
      message(STATUS "Copied dll from ${DLL_NAME} to ${DIRECTORY}")
      # Tell cmake that we need to clean up these DLLs on a "make clean"
      set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "${DIRECTORY}/${DLL_NAME}")
    endforeach(DIRECTORY)
  endforeach(DLL)
endif()

## Run a custom install script for the K4A components
## Running the two "CODE" blocks populates the cmake_install.cmake script with the information
## about which DLLs to install, and where to install them.
## We then run the more complex script to actually perform the installation.
if (${K4A_INSTALL_NEEDED})
  message("Installing K4A SDK to binary output folder")
  install(CODE "set(K4A_DLL_FILES \"${K4A_DLL_FILES}\")")
  install(CODE "set(DLL_COPY_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}\")")
  install(SCRIPT "./cmake/k4a-install.cmake")
endif()
