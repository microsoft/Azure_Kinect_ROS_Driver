// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef K4A_ROS_DEVICE_H
#define K4A_ROS_DEVICE_H

// System headers
//
#include <atomic>
#include <mutex>
#include <thread>

// Library headers
//
#include <image_transport/image_transport.hpp>
#include <k4a/k4a.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

#if defined(K4A_BODY_TRACKING)
#include <visualization_msgs/msg/marker_array.hpp>
#include <k4abt.hpp>
#endif

// Project headers
//
#include "azure_kinect_ros_driver/k4a_calibration_transform_data.h"
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"

class K4AROSDevice : public rclcpp::Node
{
 public:
  K4AROSDevice();

  ~K4AROSDevice();

  k4a_result_t startCameras();
  k4a_result_t startImu();

  void stopCameras();
  void stopImu();

  // Get camera calibration information for the depth camera
  void getDepthCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);

  void getRgbCameraInfo(sensor_msgs::msg::CameraInfo& camera_info);

  k4a_result_t getDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& depth_frame, bool rectified);

  k4a_result_t getPointCloud(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);

  k4a_result_t getRgbPointCloudInRgbFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);
  k4a_result_t getRgbPointCloudInDepthFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);

  k4a_result_t getImuFrame(const k4a_imu_sample_t& capture, std::shared_ptr<sensor_msgs::msg::Imu>& imu_frame);

  k4a_result_t getRbgFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& rgb_frame, bool rectified);
  k4a_result_t getJpegRgbFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::CompressedImage>& jpeg_image);

  k4a_result_t getIrFrame(const k4a::capture& capture, std::shared_ptr<sensor_msgs::msg::Image>& ir_image);

#if defined(K4A_BODY_TRACKING)
  k4a_result_t getBodyMarker(const k4abt_body_t& body, std::shared_ptr<visualization_msgs::msg::Marker> marker_msg, int jointType,
                             rclcpp::Time capture_time);

  k4a_result_t getBodyIndexMap(const k4abt::frame& body_frame, std::shared_ptr<sensor_msgs::msg::Image> body_index_map_image);

  k4a_result_t renderBodyIndexMapToROS(std::shared_ptr<sensor_msgs::msg::Image> body_index_map_image, k4a::image& k4a_body_index_map,
                                       const k4abt::frame& body_frame);
#endif

 private:
  k4a_result_t renderBGRA32ToROS(std::shared_ptr<sensor_msgs::msg::Image>& rgb_frame, k4a::image& k4a_bgra_frame);
  k4a_result_t renderDepthToROS(std::shared_ptr<sensor_msgs::msg::Image>& depth_image, k4a::image& k4a_depth_frame);
  k4a_result_t renderIrToROS(std::shared_ptr<sensor_msgs::msg::Image>& ir_image, k4a::image& k4a_ir_frame);

  k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);
  k4a_result_t fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image,
                                   std::shared_ptr<sensor_msgs::msg::PointCloud2>& point_cloud);

  void framePublisherThread();
#if defined(K4A_BODY_TRACKING)
  void bodyPublisherThread();
#endif
  void imuPublisherThread();

  // Gets a timestap from one of the captures images
  std::chrono::microseconds getCaptureTimestamp(const k4a::capture& capture);

  // Converts a k4a_image_t timestamp to a ros::Time object
  rclcpp::Time timestampToROS(const std::chrono::microseconds& k4a_timestamp_us);

  // Converts a k4a_imu_sample_t timestamp to a ros::Time object
  rclcpp::Time timestampToROS(const uint64_t& k4a_timestamp_us);

  // Updates the timestamp offset (stored as start_time_) between the device time and ROS time.
  // This is a low-pass filtered update based on the system time from k4a, which represents the
  // time the message arrived at the USB bus.
  void updateTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us,
                             const std::chrono::nanoseconds& k4a_system_timestamp_ns);
  // Make an initial guess based on wall clock. The best we can do when no image timestamps are
  // available.
  void initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us);

  // When using IMU throttling, computes a mean measurement from a set of IMU samples
  k4a_imu_sample_t computeMeanIMUSample(const std::vector<k4a_imu_sample_t>& samples);

  void printTimestampDebugMessage(const std::string& name, const rclcpp::Time& timestamp);


  image_transport::Publisher rgb_raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_jpeg_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_raw_camerainfo_publisher_;

  image_transport::Publisher depth_raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_raw_camerainfo_publisher_;

  image_transport::Publisher depth_rect_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_rect_camerainfo_publisher_;

  image_transport::Publisher rgb_rect_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_rect_camerainfo_publisher_;

  image_transport::Publisher ir_raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ir_raw_camerainfo_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_orientation_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

#if defined(K4A_BODY_TRACKING)
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr body_marker_publisher_;

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
  std::atomic_int16_t k4abt_tracker_queue_size_;
  std::thread body_publisher_thread_;
#endif

  std::chrono::nanoseconds device_to_realtime_offset_{0};

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

#endif  // K4A_ROS_DEVICE_H
