// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_DEVICE_H
#define K4A_ROS_DEVICE_H

// System headers
//
#include <thread>
#include <atomic>
#include <mutex>

// Library headers
//
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <image_transport/image_transport.h>

#if defined(K4A_BODY_TRACKING)
#include <k4abt.hpp>
#include <visualization_msgs/MarkerArray.h>
#endif

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"
#include "azure_kinect_ros_driver/k4a_calibration_transform_data.h"

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

    k4a_result_t getIrFrame(const k4a::capture &capture, sensor_msgs::ImagePtr ir_image);

#if defined(K4A_BODY_TRACKING)
    k4a_result_t getBodyMarker(const k4abt_body_t& body, visualization_msgs::MarkerPtr marker_msg, int jointType, ros::Time capture_time);

    k4a_result_t getBodyIndexMap(const k4abt::frame& body_frame, sensor_msgs::ImagePtr body_index_map_image);

    k4a_result_t renderBodyIndexMapToROS(sensor_msgs::ImagePtr body_index_map_image, k4a::image& k4a_body_index_map, const k4abt::frame& body_frame);
#endif

  private:
    k4a_result_t renderBGRA32ToROS(sensor_msgs::ImagePtr rgb_frame, k4a::image& k4a_bgra_frame);
    k4a_result_t renderDepthToROS(sensor_msgs::ImagePtr depth_image, k4a::image& k4a_depth_frame);
    k4a_result_t renderIrToROS(sensor_msgs::ImagePtr ir_image, k4a::image& k4a_ir_frame);

    void framePublisherThread();
    void imuPublisherThread();

    // Gets a timestap from one of the captures images
    std::chrono::microseconds getCaptureTimestamp(const k4a::capture &capture);

    // Converts a k4a_image_t timestamp to a ros::Time object
    ros::Time timestampToROS(const std::chrono::microseconds & k4a_timestamp_us);

    // Converts a k4a_imu_sample_t timestamp to a ros::Time object
    ros::Time timestampToROS(const uint64_t & k4a_timestamp_us);

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

    image_transport::Publisher  ir_raw_publisher_;
    ros::Publisher              ir_raw_camerainfo_publisher_;

    ros::Publisher              imu_orientation_publisher_;

    ros::Publisher pointcloud_publisher_;

#if defined(K4A_BODY_TRACKING)
    ros::Publisher body_marker_publisher_;

    image_transport::Publisher body_index_map_publisher_;
#endif

    // Parameters
    K4AROSDeviceParams params_;

    // K4A device
    k4a::device k4a_device_;
    K4ACalibrationTransformData calibration_data_;

    // K4A Recording
    k4a::playback k4a_playback_handle_;
    std::mutex k4a_playback_handle_mutex_;
    
#if defined(K4A_BODY_TRACKING)
    // Body tracker
    k4abt::tracker k4abt_tracker_;
#endif

    ros::Time start_time_;

    // Thread control
    volatile bool running_;

    // Last capture timestamp for synchronizing playback capture and imu thread
    std::atomic_int64_t last_capture_time_usec_;

    // Last imu timestamp for synchronizing playback capture and imu thread
    std::atomic_uint64_t last_imu_time_usec_;
    std::atomic_bool imu_stream_end_of_file_;

    // Threads
    std::thread frame_publisher_thread_;
    std::thread imu_publisher_thread_;
};

void printTimestampDebugMessage(const std::string name, const ros::Time & timestamp);

#endif // K4A_ROS_DEVICE_H