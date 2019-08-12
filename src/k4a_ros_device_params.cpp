// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"

// System headers
//

// Library headers
//
#include <k4a/k4a.h>

// Project headers
//

k4a_result_t K4AROSDeviceParams::GetDeviceConfig(k4a_device_configuration_t *configuration)
{
    configuration->color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;

    configuration->depth_delay_off_color_usec = 0;
    configuration->wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    configuration->subordinate_delay_off_master_usec = 0;
    configuration->disable_streaming_indicator = false;

    if (!color_enabled)
    {
        ROS_INFO_STREAM("Disabling RGB Camera");

        configuration->color_resolution = K4A_COLOR_RESOLUTION_OFF;
    }
    else
    {
        ROS_INFO_STREAM("Setting RGB Camera Resolution: " << color_resolution);

        if (color_resolution == "720P")
        {
            configuration->color_resolution = K4A_COLOR_RESOLUTION_720P;
        }
        else if (color_resolution == "1080P")
        {
            configuration->color_resolution = K4A_COLOR_RESOLUTION_1080P;
        }
        else if (color_resolution == "1440P")
        {
            configuration->color_resolution = K4A_COLOR_RESOLUTION_1440P;
        }
        else if (color_resolution == "1536P")
        {
            configuration->color_resolution = K4A_COLOR_RESOLUTION_1536P;
        }
        else if (color_resolution == "2160P")
        {
            configuration->color_resolution = K4A_COLOR_RESOLUTION_2160P;
        }
        else if (color_resolution == "3072P")
        {
            configuration->color_resolution = K4A_COLOR_RESOLUTION_3072P;
        }
        else
        {
            ROS_ERROR_STREAM("Invalid RGB Camera Resolution: " << color_resolution);
            return K4A_RESULT_FAILED;
        }
    }

    if (!depth_enabled)
    {
        ROS_INFO_STREAM("Disabling Depth Camera");

        configuration->depth_mode = K4A_DEPTH_MODE_OFF;
    }
    else
    {
        ROS_INFO_STREAM("Setting Depth Camera Mode: " << depth_mode);

        if (depth_mode == "NFOV_2X2BINNED")
        {
            configuration->depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
        }
        else if (depth_mode == "NFOV_UNBINNED")
        {
            configuration->depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        }
        else if (depth_mode == "WFOV_2X2BINNED")
        {
            configuration->depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        }
        else if (depth_mode == "WFOV_UNBINNED")
        {
            configuration->depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
        }
        else if (depth_mode == "PASSIVE_IR")
        {
            configuration->depth_mode = K4A_DEPTH_MODE_PASSIVE_IR;
        }
        else
        {
            ROS_ERROR_STREAM("Invalid Depth Camera Mode: " << depth_mode);
            return K4A_RESULT_FAILED;
        }
    }

    ROS_INFO_STREAM("Setting Camera FPS: " << fps);

    if (fps == 5)
    {
        configuration->camera_fps = K4A_FRAMES_PER_SECOND_5;
    }
    else if (fps == 15)
    {
        configuration->camera_fps = K4A_FRAMES_PER_SECOND_15;
    }
    else if (fps == 30)
    {
        configuration->camera_fps = K4A_FRAMES_PER_SECOND_30;
    }
    else
    {
        ROS_ERROR_STREAM("Invalid Camera FPS: " << fps);
        return K4A_RESULT_FAILED;
    }

    // Ensure that if RGB and depth cameras are enabled, we ask for synchronized frames
    if (depth_enabled && color_enabled)
    {
        configuration->synchronized_images_only = true;
    }
    else
    {
        configuration->synchronized_images_only = false;
    }

    // Ensure that the "point_cloud" option is not used with passive IR mode, since they are incompatible
    if (point_cloud && (configuration->depth_mode == K4A_DEPTH_MODE_PASSIVE_IR))
    {
        ROS_ERROR_STREAM("Incompatible options: cannot generate point cloud if depth camera is using PASSIVE_IR mode.");
        return K4A_RESULT_FAILED;
    }

    // Ensure that point_cloud is enabled if using rgb_point_cloud
    if (rgb_point_cloud && !point_cloud)
    {
        ROS_ERROR_STREAM("Incompatible options: cannot generate RGB point cloud if point_cloud is not enabled.");
        return K4A_RESULT_FAILED;
    }

    // Ensure that color camera is enabled when generating a color point cloud
    if (rgb_point_cloud && !color_enabled)
    {
        ROS_ERROR_STREAM("Incompatible options: cannot generate RGB point cloud if color camera is not enabled.");
        return K4A_RESULT_FAILED;
    }

    return K4A_RESULT_SUCCEEDED;
}

void K4AROSDeviceParams::Help()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    ROS_INFO("#param_variable - #param_type : param_help_string (#param_default_val)");

    ROS_PARAM_LIST
#undef LIST_ENTRY
}

void K4AROSDeviceParams::Print()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
    ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

    ROS_PARAM_LIST
#undef LIST_ENTRY
}