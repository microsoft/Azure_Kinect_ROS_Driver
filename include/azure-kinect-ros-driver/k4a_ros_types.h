// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_TYPES_H
#define K4A_ROS_TYPES_H

using DepthPixel = uint16_t;

struct BgraPixel
{
    uint8_t Blue;
    uint8_t Green;
    uint8_t Red;
    uint8_t Alpha;
};

#endif // K4A_ROS_TYPES_H