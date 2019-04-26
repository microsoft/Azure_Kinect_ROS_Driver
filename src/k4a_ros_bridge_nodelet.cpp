// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure-kinect-ros-driver/k4a_ros_bridge_nodelet.h"

// System headers
//

// Library headers
//

// Project headers
//

K4AROSBridgeNodelet::K4AROSBridgeNodelet() : Nodelet(),
                                             k4a_device(nullptr)
{
}

K4AROSBridgeNodelet::~K4AROSBridgeNodelet()
{
}

void K4AROSBridgeNodelet::onInit()
{
    k4a_device = std::unique_ptr<K4AROSDevice>(new K4AROSDevice(getNodeHandle(), getPrivateNodeHandle()));

    if (!k4a_device->startCameras())
    {
        k4a_device.reset(nullptr);
        throw nodelet::Exception("Could not start K4A_ROS_Device!");
    }
}
