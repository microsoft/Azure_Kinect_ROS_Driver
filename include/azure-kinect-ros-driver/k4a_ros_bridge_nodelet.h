// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_BRIDGE_NODELET_H
#define K4A_ROS_BRIDGE_NODELET_H

// System headers
//

// Library headers
//
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Project headers
//
#include "azure-kinect-ros-driver/k4a_ros_device.h"

class K4AROSBridgeNodelet : public nodelet::Nodelet
{
public:

    K4AROSBridgeNodelet();
    ~K4AROSBridgeNodelet();

    virtual void onInit();

private:

    std::unique_ptr<K4AROSDevice> k4a_device;
};

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(K4AROSBridgeNodelet, nodelet::Nodelet)

#endif // K4A_ROS_BRIDGE_NODELET_H