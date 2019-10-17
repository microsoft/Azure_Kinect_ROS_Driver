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

/** Three dimensional double precision floating point vector.
 *
 * \xmlonly
 * <requirements>
 *   <requirement name="Header">k4atypes.h (include k4a/k4a.h)</requirement>
 * </requirements>
 * \endxmlonly
 */
typedef union
{
    /** XYZ or array representation of vector. */
    struct _xyz
    {
        double x; /**< X component of a vector. */
        double y; /**< Y component of a vector. */
        double z; /**< Z component of a vector. */
    } xyz;       /**< X, Y, Z representation of a vector. */
    double v[3];  /**< Array representation of a vector. */
} k4a_double3_t;

/** IMU sample in double precision.
 *
 * \xmlonly
 * <requirements>
 *   <requirement name="Header">k4atypes.h (include k4a/k4a.h)</requirement>
 * </requirements>
 * \endxmlonly
 */
typedef struct _k4a_imu_sample_double_t
{
    double temperature;            /**< Temperature reading of this sample (Celsius). */
    k4a_double3_t acc_sample;      /**< Accelerometer sample in meters per second squared. */
    uint64_t acc_timestamp_usec;  /**< Timestamp of the accelerometer in microseconds. */
    k4a_double3_t gyro_sample;     /**< Gyro sample in radians per second. */
    uint64_t gyro_timestamp_usec; /**< Timestamp of the gyroscope in microseconds */
} k4a_imu_sample_double_t;

#endif // K4A_ROS_TYPES_H