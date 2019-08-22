// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_DEVICE_PARAMS_H
#define K4A_ROS_DEVICE_PARAMS_H

// System headers
//

// Library headers
//
#include <k4a/k4a.h>
#include <ros/ros.h>

// Project headers
//


// The format of these list entries is :
//
// LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)
//
// param_variable: the variable name which will be created in the k4a_ros_device class to hold the contents of the parameter
// param_help_string: a string containing help information for the parameter
// param_type: the type of the parameter
// param_default_val: the default value of the parameter
//
// Example:
// LIST_ENTRY(sensor_sn, "The serial number of the sensor this node should connect with", std::string, std::string(""))
#define ROS_PARAM_LIST \
    LIST_ENTRY(sensor_sn, "The serial number of the sensor this node should connect to.", std::string, std::string("")) \
    LIST_ENTRY(depth_enabled, "True if depth camera should be enabled", bool, true) \
    LIST_ENTRY(depth_mode, "The mode of the depth camera. Options are: NFOV_2X2BINNED, NFOV_UNBINNED, WFOV_2X2BINNED, WFOV_UNBINNED, PASSIVE_IR", std::string, std::string("NFOV_UNBINNED")) \
    LIST_ENTRY(color_enabled, "True if color camera should be enabled", bool, false) \
    LIST_ENTRY(color_resolution, "The resolution of the color camera. Options are: 720P, 1080P, 1440P, 1536P, 2160P, 3072P", std::string, std::string("720P")) \
    LIST_ENTRY(fps, "The FPS of the RGB and Depth cameras. Options are: 5, 15, 30", int, 5) \
    LIST_ENTRY(point_cloud, "A PointCloud2 based on depth data. Requires depth_enabled=true, and cannot be used with depth_mode=PASSIVE_IR", bool, true) \
    LIST_ENTRY(rgb_point_cloud, "Add RGB camera data to the point cloud. Requires point_cloud=true and color_enabled=true", bool, false) \
    LIST_ENTRY(tf_prefix, "The prefix prepended to tf frame ID's", std::string, std::string()) \
    LIST_ENTRY(recording_file, "Path to a recording file to open instead of opening a device", std::string, std::string("")) \
    LIST_ENTRY(recording_loop_enabled, "True if the recording should be rewound at EOF", bool, false) \

class K4AROSDeviceParams
{
public:

    // Get a device configuration from a a set of parameters
    k4a_result_t GetDeviceConfig(k4a_device_configuration_t* configuration);

    // Print help messages to the console
    void Help();

    // Print the value of all parameters
    void Print();

    // Parameters
    #define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
        ROS_PARAM_LIST
    #undef LIST_ENTRY
};

#endif // K4A_ROS_DEVICE_PARAMS_H