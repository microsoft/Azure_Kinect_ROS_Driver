function(print_variable x)
    message(STATUS "${x}: ${${x}}")
endfunction()

function(quiet_message)
    if(NOT FIND_QUIETLY)
        message(${ARGV})
    endif()
endfunction()

## Initialize some standardized variables
set(FIND_VERSION_COUNT ${${CMAKE_FIND_PACKAGE_NAME}_FIND_VERSION_COUNT})
set(FIND_VERSION_EXACT ${${CMAKE_FIND_PACKAGE_NAME}_FIND_VERSION_EXACT})
set(FIND_VERSION ${${CMAKE_FIND_PACKAGE_NAME}_FIND_VERSION})
set(FIND_VERSION_MAJOR ${${CMAKE_FIND_PACKAGE_NAME}_FIND_VERSION_MAJOR})
set(FIND_VERSION_MINOR ${${CMAKE_FIND_PACKAGE_NAME}_FIND_VERSION_MINOR})
set(FIND_VERSION_PATCH ${${CMAKE_FIND_PACKAGE_NAME}_FIND_VERSION_PATCH})
set(FIND_QUIETLY ${${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY})
set(FIND_REQUIRED ${${CMAKE_FIND_PACKAGE_NAME}_FIND_REQUIRED})

set(RELATIVE_WIN_LIB_DIR "sdk/windows-desktop/amd64/release/lib")
set(RELATIVE_WIN_BIN_DIR "sdk/windows-desktop/amd64/release/bin")

set(RELATIVE_WIN_K4A_LIB_PATH "${RELATIVE_WIN_LIB_DIR}/k4a.lib")
set(RELATIVE_WIN_K4A_DLL_PATH "${RELATIVE_WIN_BIN_DIR}/k4a.dll")

set(RELATIVE_WIN_K4ARECORD_LIB_PATH "${RELATIVE_WIN_LIB_DIR}/k4arecord.lib")
set(RELATIVE_WIN_K4ARECORD_DLL_PATH "${RELATIVE_WIN_BIN_DIR}/k4arecord.dll")

if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(DEPTHENGINE_GLOB "depthengine_*.dll")
elseif(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(DEPTHENGINE_GLOB "libdepthengine.so.*")
else()
    message(FATAL_ERROR "Unknown CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
endif()

# K4A versions have exactly 3 components: major.minor.rev
if (NOT (FIND_VERSION_COUNT EQUAL 3))
    message(FATAL_ERROR "Error: Azure Kinect SDK Version numbers contain exactly 3 components (major.minor.rev). Requested number of components: ${FIND_VERSION_COUNT}")
endif()

# First, check the ext/sdk folder. Always do this first so that we can do platform-dependent work afterwards
find_package(k4a ${FIND_VERSION} ${_exact_arg} ${_quiet_arg} NO_MODULE NO_DEFAULT_PATH PATHS "${PROJECT_SOURCE_DIR}/ext/sdk")
find_package(k4arecord ${FIND_VERSION} ${_exact_arg} ${_quiet_arg} NO_MODULE NO_DEFAULT_PATH PATHS "${PROJECT_SOURCE_DIR}/ext/sdk")

if(${k4a_FOUND} AND ${k4arecord_FOUND})
    set(K4A_INSTALL_NEEDED TRUE)

    quiet_message(STATUS "Found an Azure Kinect SDK in ./ext/sdk")

    # Add the depth engine as an IMPORTED_LINK_DEPENDENT_LIBRARIES to ensure it gets copied
    unset(_depthengine_bin)
    file(GLOB_RECURSE _depthengine_bin 
        LIST_DIRECTORIES FALSE 
        "${PROJECT_SOURCE_DIR}/ext/sdk/*${DEPTHENGINE_GLOB}")

    if(NOT _depthengine_bin)
        quiet_message(WARNING "Could not find depth engine in ./ext/sdk! Rejecting ext/sdk")
    else()
        set_property(TARGET k4a PROPERTY IMPORTED_LINK_DEPENDENT_LIBRARIES "${_depthengine_bin}")

        # If we found a valid SDK in ext/sdk, this always overrides anything we might find in the system path.
        # k4a_FOUND should be set by find_package(k4a), so return now.
        quiet_message(STATUS "Accepted SDK in ./ext/sdk. System paths will not be searched")
        return()
    endif()
endif()

# On Linux, we might find k4a installed to the system path. Failing that, we can search the ext/sdk folder.
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    
    set(K4A_INSTALL_NEEDED FALSE)

    set(_exact_arg )
    if (FIND_VERSION_EXACT)
        set(_exact_arg EXACT)
    endif()

    set(_quiet_arg )
    if (FIND_QUIETLY)
        set(_quiet_arg QUIET)
    endif()

    set(_required_arg )
    if (FIND_REQUIRED)
        set(_required_arg REQUIRED)
    endif()

    # Linux is much easier: just check if k4a is installed to the system path
    find_package(k4a ${FIND_VERSION} ${_exact_arg} ${_quiet_arg} ${_required_arg} NO_MODULE)
    find_package(k4arecord ${FIND_VERSION} ${_exact_arg} ${_quiet_arg} ${_required_arg} NO_MODULE)

# On Windows, we will have to find K4A installed in Program Files. Failing that, we can search the ext/sdk folder.
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
    
    # Windows always needs installation
    set(K4A_INSTALL_NEEDED TRUE)

    # Get a list of SDK's installed in Program Files
    file(GLOB _sdk_dirs "C:/Program Files/Azure Kinect SDK*")

    set(_best_sdk_dir "")
    set(_best_sdk_version "0.0.0")

    foreach(_sdk_dir ${_sdk_dirs})

        unset(_version_file CACHE)
        find_file(_version_file "version.txt"
            PATHS ${_sdk_dir}
            NO_DEFAULT_PATH
        )

        if(NOT _version_file)
            quiet_message(WARNING "Azure Kinect SDK located at ${_sdk_dir} does not contain a version file. Skipping this SDK.")
            continue()
        endif()

        set(_version_file_contents "0.0.0")
        file(READ ${_version_file} _version_file_contents)

        if(${_version_file_contents} MATCHES "^([0-9]+)[.]([0-9]+)[.]([0-9]+)(-.+)?")
            set(_sdk_version "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}.${CMAKE_MATCH_3}")
            set(_sdk_version_major "${CMAKE_MATCH_1}")
            set(_sdk_version_minor "${CMAKE_MATCH_2}")
            set(_sdk_version_patch "${CMAKE_MATCH_3}")

            quiet_message("Found Azure Kinect SDK located at ${_sdk_dir} with version ${_sdk_version}")

            # Now do the real logic. First, exact matches should take precedence (if requested)
            if (FIND_VERSION_EXACT)
                # If we found the exact version requested, no need to search any more
                if (_sdk_version VERSION_EQUAL FIND_VERSION)
                    set(_best_sdk_dir ${_sdk_dir})
                    set(_best_sdk_version ${_sdk_version})
                    break()
                endif()
            else()
                # Find the "best compatible version"
                # For K4A, this means:
                # - Major version must be the same as requested
                # - Minor version must be greater-than-or-equal requested
                # - Patch version must be greater-than-or-equal requested
                
                # First check if this SDK is major-version compatible
                if (NOT (_sdk_version_major EQUAL FIND_VERSION_MAJOR))
                    quiet_message(STATUS "Rejecting SDK located at ${_sdk_dir}: Major version mismatch (found ${_sdk_version_major}, requested ${FIND_VERSION_MAJOR})")
                    continue()
                endif()

                # Next, check if minor version is greater than or equal requested
                if (NOT (_sdk_version_minor GREATER_EQUAL FIND_VERSION_MINOR))
                    quiet_message(STATUS "Rejecting SDK located at ${_sdk_dir}: Minor version too low (found ${_sdk_version_minor}, requested at least ${FIND_VERSION_MINOR})")
                    continue()
                endif()

                # Finally, check if patch version is greater than or equal requested
                if (NOT (_sdk_version_patch GREATER_EQUAL FIND_VERSION_PATCH))
                    quiet_message(STATUS "Rejecting SDK located at ${_sdk_dir}: Patch version too low (found ${_sdk_version_patch}, requested at least ${FIND_VERSION_PATCH})")
                    continue()
                endif()

                # If we got here, the SDK is version compatible. Check if a better SDK version has already been selected
                if (_best_sdk_version VERSION_GREATER _sdk_version)
                    quiet_message(STATUS "Rejecting SDK located at ${_sdk_dir}: A more recent SDK has already been found (found ${_sdk_version}, already found ${_best_sdk_version})")
                    continue()
                endif()

                set(_k4a_lib_path "${_sdk_dir}/${RELATIVE_WIN_K4A_LIB_PATH}")
                if(NOT EXISTS "${_k4a_lib_path}")
                    quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find k4a.lib at ${_k4a_lib_path}")
                    continue()
                endif()

                set(_k4arecord_lib_path "${_sdk_dir}/${RELATIVE_WIN_K4ARECORD_LIB_PATH}")
                if(NOT EXISTS "${_k4arecord_lib_path}")
                    quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find k4arecord.lib at ${_k4arecord_lib_path}")
                    continue()
                endif()

                set(_k4a_bin_path "${_sdk_dir}/${RELATIVE_WIN_K4A_DLL_PATH}")
                if(NOT EXISTS "${_k4a_bin_path}")
                    quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find k4a.dll at ${_k4a_bin_path}")
                    continue()
                endif()

                set(_k4arecord_bin_path "${_sdk_dir}/${RELATIVE_WIN_K4ARECORD_DLL_PATH}")
                if(NOT EXISTS "${_k4arecord_bin_path}")
                    quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find k4arecord.dll at ${_k4arecord_bin_path}")
                    continue()
                endif()

                unset(_depthengine_bin)
                file(GLOB _depthengine_bin 
                    LIST_DIRECTORIES FALSE 
                    ${_sdk_dir}/${RELATIVE_WIN_BIN_DIR}/${DEPTHENGINE_GLOB})

                if(NOT _depthengine_bin)
                    quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find a depth engine dll at ${_sdk_dir}/${RELATIVE_WIN_BIN_DIR}")
                    continue()
                endif()

                set(_best_sdk_version ${_sdk_version})
                set(_best_sdk_dir ${_sdk_dir})

            endif()

        else()
            quiet_message(WARNING "Could not parse SDK version found in ${_version_file}. Found version ${_version_file_contents}. Skipping this SDK.")
            continue()
        endif()

    endforeach()
    
    if (_best_sdk_version VERSION_EQUAL "0.0.0")

        set(_message_type WARNING)
        if (FIND_REQUIRED)
            set(_message_type FATAL_ERROR)
        endif()
        
        # Print this message if FIND_REQURIED since it is mandatory to produce a fatal error, or 
        # print a warning if we weren't asked to be quiet
        if(FIND_REQUIRED OR (NOT FIND_QUIETLY))
            message(${_message_type} "Could not find a compatible Azure Kinect Sensor SDK installed in Program Files")
        endif()

        return()
    endif()

    add_library(k4a SHARED IMPORTED GLOBAL)
    add_library(k4a::k4a ALIAS k4a)

    target_include_directories(
        k4a 
        INTERFACE
            ${_best_sdk_dir}/sdk/include
            ${_best_sdk_dir}/sdk/include/k4a
    )

    set_property(TARGET k4a PROPERTY IMPORTED_LOCATION "${_best_sdk_dir}/${RELATIVE_WIN_K4A_DLL_PATH}")
    set_property(TARGET k4a PROPERTY IMPORTED_IMPLIB "${_best_sdk_dir}/${RELATIVE_WIN_K4A_LIB_PATH}")
    
    unset(_depthengine_bin)
    file(GLOB _depthengine_bin 
        LIST_DIRECTORIES FALSE 
        ${_best_sdk_dir}/${RELATIVE_WIN_BIN_DIR}/${DEPTHENGINE_GLOB})

    if(NOT _depthengine_bin)
        message(FATAL_ERROR "SDK was accepted without a depth engine!")
        return()
    endif()

    # Mark the depthengine as a requirement for running k4a.dll
    set_property(TARGET k4a PROPERTY IMPORTED_LINK_DEPENDENT_LIBRARIES "${_depthengine_bin}")
    
    add_library(k4arecord SHARED IMPORTED GLOBAL)
    add_library(k4a::k4arecord ALIAS k4arecord)
    set_property(TARGET k4arecord PROPERTY IMPORTED_LOCATION "${_best_sdk_dir}/${RELATIVE_WIN_K4ARECORD_DLL_PATH}")
    set_property(TARGET k4arecord PROPERTY IMPORTED_IMPLIB "${_best_sdk_dir}/${RELATIVE_WIN_K4ARECORD_LIB_PATH}")
    target_include_directories(
        k4a 
        INTERFACE
            ${_best_sdk_dir}/sdk/include
            ${_best_sdk_dir}/sdk/include/k4arecord
    )

    set(${CMAKE_FIND_PACKAGE_NAME}_FOUND TRUE)

else()
    message(FATAL_ERROR "Unknown host system: ${CMAKE_HOST_SYSTEM_NAME}")
endif()
