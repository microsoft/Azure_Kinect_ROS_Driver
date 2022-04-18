include(FindModuleHelpers)

set(RELATIVE_WIN_LIB_DIR "sdk/windows-desktop/amd64/release/lib")
set(RELATIVE_WIN_BIN_DIR "sdk/windows-desktop/amd64/release/bin")

set(RELATIVE_WIN_K4ABT_LIB_PATH "${RELATIVE_WIN_LIB_DIR}/k4abt.lib")

set(RELATIVE_WIN_K4ABT_DLL_PATH "${RELATIVE_WIN_BIN_DIR}/k4abt.dll")
set(RELATIVE_WIN_DNN_MODEL_PATH "${RELATIVE_WIN_BIN_DIR}/dnn_model_2_0.onnx")
set(RELATIVE_WIN_ONNX_RUNTIME_DLL_PATH "${RELATIVE_WIN_BIN_DIR}/onnxruntime.dll")

# K4A BT versions have exactly 3 components: major.minor.rev
if (NOT (FIND_VERSION_COUNT EQUAL 3))
    message(FATAL_ERROR "Error: Azure Kinect Body Tracking SDK Version numbers contain exactly 3 components (major.minor.rev). Requested number of components: ${FIND_VERSION_COUNT}")
endif()

# On Linux, we should find k4abt installed to the system path
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")

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

    find_package(k4abt ${FIND_VERSION} ${_exact_arg} ${_quiet_arg} ${_required_arg} NO_MODULE)

# On Windows, we will have to find K4A Body Tracking installed in Program Files.
# We also cannot do any version checking on Windows
elseif(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
    
    # Windows always needs installation
    set(K4A_INSTALL_NEEDED TRUE)

    set(_sdk_dir "C:/Program Files/Azure Kinect Body Tracking SDK")

    # Get a list of SDK's installed in Program Files
    if(EXISTS ${_sdk_dir})
        set(_k4abt_lib_path "${_sdk_dir}/${RELATIVE_WIN_K4ABT_LIB_PATH}")
        if(NOT EXISTS "${_k4abt_lib_path}")
            quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find k4abt.lib at ${_k4abt_lib_path}")
            return()
        endif()

        set(_k4abt_bin_path "${_sdk_dir}/${RELATIVE_WIN_K4ABT_DLL_PATH}")
        if(NOT EXISTS "${_k4abt_bin_path}")
            quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find k4abt.dll at ${_k4abt_bin_path}")
            return()
        endif()

        set(_dnn_model_path "${_sdk_dir}/${RELATIVE_WIN_DNN_MODEL_PATH}")
        if(NOT EXISTS "${_dnn_model_path}")
            quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find dnn_model_2_0.onnx at ${_dnn_model_path}")
            return()
        endif()

        set(_onnx_runtime_bin_path "${_sdk_dir}/${RELATIVE_WIN_ONNX_RUNTIME_DLL_PATH}")
        if(NOT EXISTS "${_onnx_runtime_bin_path}")
            quiet_message(WARNING "Rejecting SDK located at ${_sdk_dir}: Could not find onnxruntime.dll at ${_onnx_runtime_bin_path}")
            return()
        endif()

        if (FIND_EXACT)
            message(FATAL_ERROR "Cannot find EXACT versions of the Azure Kinect Body Tracking SDK on Windows as no version information is available.")
        endif()

        add_library(k4abt::k4abt SHARED IMPORTED GLOBAL)

        target_include_directories(
            k4abt::k4abt
            INTERFACE
                ${_sdk_dir}/sdk/include
        )

        set_property(TARGET k4abt::k4abt PROPERTY IMPORTED_CONFIGURATIONS "")
        set_property(TARGET k4abt::k4abt PROPERTY IMPORTED_LOCATION "${_k4abt_bin_path}")
        set_property(TARGET k4abt::k4abt PROPERTY IMPORTED_IMPLIB "${_k4abt_lib_path}")
        
        # Mark the depthengine as a requirement for running k4a.dll
        set_property(TARGET k4abt::k4abt PROPERTY IMPORTED_LINK_DEPENDENT_LIBRARIES "${_dnn_model_path};${_onnx_runtime_bin_path}")
                
        set(${CMAKE_FIND_PACKAGE_NAME}_FOUND TRUE)
        set(${CMAKE_FIND_PACKAGE_NAME}_VERSION ${FIND_VERSION})
    else()

        set(_message_type WARNING)
        if (FIND_REQUIRED)
            set(_message_type FATAL_ERROR)
        endif()
        
        # Print this message if FIND_REQURIED since it is mandatory to produce a fatal error, or 
        # print a warning if we weren't asked to be quiet
        if(FIND_REQUIRED OR (NOT FIND_QUIETLY))
            message(${_message_type} "Could not find a compatible Azure Kinect Body Tracking SDK installed in Program Files")
        endif()

        return()
    endif()

else()
    message(FATAL_ERROR "Unknown host system: ${CMAKE_HOST_SYSTEM_NAME}")
endif()
