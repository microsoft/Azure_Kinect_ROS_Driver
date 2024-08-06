// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_color_params.h"

// System headers
//

// Library headers
//
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

// Project headers
//

K4AROSColorParams::K4AROSColorParams() : rclcpp::Node("k4a_ros_color_params_node") {}

bool K4AROSColorParams::SetColorConfig(k4a::device &device)
{
  if (exposure_auto)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting exposure mode to AUTO");
    device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting exposure mode to MANUAL and exposure time: " << exposure_time);
    device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL, exposure_time);
  }

  if (whitebalance_auto)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting whitebalance mode to AUTO");
    device.set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_AUTO, 0);
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting whitebalance mode to MANUAL and value: " << whitebalance_val);

    if (whitebalance_val >= 2500 && whitebalance_val <= 12500 && whitebalance_val % 10 == 0)
      device.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_MANUAL,
                               whitebalance_val);
    else
      RCLCPP_ERROR_STREAM(this->get_logger(), "Whitebalance value not valid: " << whitebalance_val
                                                                               << "\n Value must be between 2500 and 12500 and evenly divisible by 10");
  }

  if (brightness >= 0 && brightness <= 255)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting brightness to: " << brightness);
    device.set_color_control(K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, brightness);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Brightness value not valid: " << brightness << "\n Value must be between 0 and 255");
  }

  if (contrast >= 0 && contrast <= 10)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting contrast to: " << contrast);
    device.set_color_control(K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, contrast);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Contrast value not valid: " << contrast << "\n Value must be between 0 and 10");
  }

  if (saturation >= 0 && contrast <= 63)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting saturation to: " << saturation);
    device.set_color_control(K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, saturation);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Saturation value not valid: " << saturation << "\n Value must be between 0 and 63");
  }

  if (sharpness >= 0 && sharpness <= 4)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting sharpness to: " << sharpness);
    device.set_color_control(K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, sharpness);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Sharpness value not valid: " << sharpness << "\n Value must be between 0 and 4");
  }

  if (gain >= 0 && gain <= 255)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting gain to: " << gain);
    device.set_color_control(K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, gain);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Gain value not valid: " << gain << "\n Value must be between 0 and 255");
  }

  if (backlight_compensation)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting backlight compensation to TRUE");
    device.set_color_control(K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, 1);
  }
  else
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting backlight compensation to FALSE");
    device.set_color_control(K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0);
  }

  if (powerline_frequency == 1)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting powerline frequency to 50 Hz");
    device.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, 1);
  }
  else if (powerline_frequency == 2)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Setting powerline frequency to 60 Hz");
    device.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, 2);
  }
  else
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Powerline frequency must be either 1 (50 Hz) or 2 (60Hz), and was set as: "
                                                << powerline_frequency);
  }

  return true;
}

void K4AROSColorParams::Help()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  RCLCPP_INFO(this->get_logger(),"#param_variable - #param_type : param_help_string (#param_default_val)");

  ROS_PARAM_LIST_COLOR
#undef LIST_ENTRY
}

void K4AROSColorParams::Print()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  RCLCPP_INFO_STREAM(this->get_logger(),"" << #param_variable << " - " << #param_type " : " << param_variable);

  ROS_PARAM_LIST_COLOR
#undef LIST_ENTRY
}
