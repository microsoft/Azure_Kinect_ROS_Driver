// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_TYPES_H
#define K4A_ROS_TYPES_H

using DepthPixel = uint16_t;
using IrPixel = uint16_t;

struct BgraPixel
{
    uint8_t Blue;
    uint8_t Green;
    uint8_t Red;
    uint8_t Alpha;
};

#if defined(K4A_BODY_TRACKING)
using BodyIndexMapPixel = uint8_t;

struct Color
{
    float r, g, b, a;
};

using ColorPalette = std::array<Color, 8>;
// a palette of 8 colors to colorize the different body markers and the body index map
static const ColorPalette BODY_COLOR_PALETTE{{
    {1.0f, 0.0f, 0.0f, 1.0f},
    {0.0f, 1.0f, 0.0f, 1.0f},
    {0.0f, 0.0f, 1.0f, 1.0f},
    {1.0f, 1.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 1.0f, 1.0f},
    {0.0f, 1.0f, 1.0f, 1.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 1.0f, 1.0f, 1.0f}
}};
#endif

#endif // K4A_ROS_TYPES_H