// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_COLOR_PARAMS_H
#define K4A_ROS_COLOR_PARAMS_H

// System headers
//

// Library headers
//
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include "rclcpp/rclcpp.hpp"
// Project headers
//

// The format of these list entries is :
//
// LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)
//
// param_variable: the variable name which will be created in the k4a_ros_device class to hold the contents of the
//    parameter
// param_help_string: a string containing help information for the parameter
// param_type: the type of the parameter
// param_default_val: the default value of the parameter
//
// Example:
// LIST_ENTRY(sensor_sn, "The serial number of the sensor this node should connect with", std::string, std::string(""))
#define ROS_PARAM_LIST_COLOR                                                                                           \
  LIST_ENTRY(exposure_auto, "True if using auto exposure time", bool, true)                                            \
  LIST_ENTRY(exposure_time, "Exposure time is measured in microseconds", int, 16670)                                   \
  LIST_ENTRY(whitebalance_auto, "True if using auto whitebalance", bool, true)                                         \
  LIST_ENTRY(whitebalance_val, "The unit is degrees Kelvin."                                                           \
                                "The setting must be set to a value evenly divisible by 10 degrees, "                  \
                                "between 2500 and 12500.", int, 4500)                                                  \
  LIST_ENTRY(brightness, "Brightness value, the valid range is 0 to 255.", int, 128)                                   \
  LIST_ENTRY(contrast, "Contrast value, the valid range is 0 to 10", int, 5)                                           \
  LIST_ENTRY(saturation, "Saturation value, the valid range is 0 to 63", int, 32)                                      \
  LIST_ENTRY(sharpness, "Sharpness value, the valid range is 0 to 4", int, 2)                                          \
  LIST_ENTRY(gain, "Gain value, the valid range is 0 to 255", int, 128)                                                \
  LIST_ENTRY(powerline_frequency, "Value of 1 sets the powerline compensation to 50 Hz. "                              \
                                  "Value of 2 sets the powerline compensation to 60 Hz.", int, 1)                      \
  LIST_ENTRY(backlight_compensation, "Backlight compensation enable", bool, false)

class K4AROSColorParams : public rclcpp::Node
{
public:
  K4AROSColorParams();

  // Set color config from a set of parameters
  bool SetColorConfig(k4a::device &device);

  // Print help messages to the console
  void Help();

  // Print the value of all parameters
  void Print();

// Parameters
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
  ROS_PARAM_LIST_COLOR
#undef LIST_ENTRY
};

#endif  // K4A_ROS_COLOR_PARAMS_H
