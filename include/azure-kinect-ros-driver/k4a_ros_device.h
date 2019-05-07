// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_DEVICE_H
#define K4A_ROS_DEVICE_H

// System headers
//
#include <thread>

// Library headers
//
#include <k4a/k4a.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <image_transport/image_transport.h>

// Project headers
//
#include "azure-kinect-ros-driver/k4a_ros_device_params.h"
#include "azure-kinect-ros-driver/k4a_calibration_transform_data.h"
#include "azure-kinect-ros-driver/k4a.hpp"

class K4AROSDevice
{
  public:
    K4AROSDevice(const ros::NodeHandle &n = ros::NodeHandle(), const ros::NodeHandle &p = ros::NodeHandle("~"));

    ~K4AROSDevice();

    k4a_result_t startCameras();
    k4a_result_t startImu();

    void stopCameras();
    void stopImu();

    // Get camera calibration information for the depth camera
    void getDepthCameraInfo(sensor_msgs::CameraInfo &camera_info);
    
    void getRgbCameraInfo(sensor_msgs::CameraInfo &camera_info);

    k4a_result_t getDepthFrame(const k4a::capture &capture, sensor_msgs::ImagePtr depth_frame, bool rectified);

    k4a_result_t getPointCloud(const k4a::capture &capture, sensor_msgs::PointCloud2Ptr point_cloud);

    k4a_result_t getRgbPointCloud(const k4a::capture &capture, sensor_msgs::PointCloud2Ptr point_cloud);

    k4a_result_t getImuFrame(const k4a_imu_sample_t &capture, sensor_msgs::ImuPtr imu_frame);

    k4a_result_t getRbgFrame(const k4a::capture &capture, sensor_msgs::ImagePtr rgb_frame, bool rectified);

  private:
    k4a_result_t renderBGRA32ToROS(sensor_msgs::ImagePtr rgb_frame, k4a::image& k4a_bgra_frame);
    k4a_result_t renderDepthToROS(sensor_msgs::ImagePtr depth_image, k4a::image& k4a_depth_frame);

    void framePublisherThread();
    void imuPublisherThread();

    // ROS Node variables
    ros::NodeHandle node_;
    ros::NodeHandle private_node_;

    image_transport::ImageTransport image_transport_;

    image_transport::Publisher  rgb_raw_publisher_;
    ros::Publisher              rgb_raw_camerainfo_publisher_;

    image_transport::Publisher  depth_raw_publisher_;
    ros::Publisher              depth_raw_camerainfo_publisher_;

    image_transport::Publisher  depth_rect_publisher_;
    ros::Publisher              depth_rect_camerainfo_publisher_;

    image_transport::Publisher  rgb_rect_publisher_;
    ros::Publisher              rgb_rect_camerainfo_publisher_;

    ros::Publisher              imu_orientation_publisher_;
    ros::Publisher              imu_temperature_publisher_;

    ros::Publisher pointcloud_publisher_;

    // Parameters
    K4AROSDeviceParams params_;

    // K4A device
    k4a::device k4a_device_;
    K4ACalibrationTransformData calibration_data_;

    ros::Time start_time_;

    // Thread control
    volatile bool running_;

    // Threads
    std::thread frame_publisher_thread_;
    std::thread imu_publisher_thread_;
};

#endif // K4A_ROS_DEVICE_H