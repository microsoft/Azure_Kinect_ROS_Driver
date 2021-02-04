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
#include <image_transport/image_transport.h>
#include <k4a/k4a.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>
#include <camera_info_manager/camera_info_manager.h>

#if defined(K4A_BODY_TRACKING)
#include <visualization_msgs/MarkerArray.h>
#include <k4abt.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

// Project headers
//
#include "azure_kinect_ros_driver/k4a_calibration_transform_data.h"
#include "azure_kinect_ros_driver/k4a_ros_device_params.h"

class K4AROSDevice
{
 public:
  K4AROSDevice(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));

  ~K4AROSDevice();

  k4a_result_t startCameras();
  k4a_result_t startImu();

  void stopCameras();
  void stopImu();

  k4a_result_t getDepthFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& depth_frame, bool rectified);

  k4a_result_t getPointCloud(const k4a::capture& capture, sensor_msgs::PointCloud2Ptr& point_cloud);

  k4a_result_t getRgbPointCloudInRgbFrame(const k4a::capture& capture, sensor_msgs::PointCloud2Ptr& point_cloud);
  k4a_result_t getRgbPointCloudInDepthFrame(const k4a::capture& capture, sensor_msgs::PointCloud2Ptr& point_cloud);

  k4a_result_t getImuFrame(const k4a_imu_sample_t& capture, sensor_msgs::ImuPtr& imu_frame);

  k4a_result_t getRbgFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& rgb_frame, bool rectified);
  k4a_result_t getJpegRgbFrame(const k4a::capture& capture, sensor_msgs::CompressedImagePtr& jpeg_image);

  k4a_result_t getIrFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& ir_image);

#if defined(K4A_BODY_TRACKING)
k4a_result_t getBodyMarker(const k4abt_body_t& body, visualization_msgs::MarkerPtr marker_msg, geometry_msgs::TransformStamped& transform_msg, int bodyNum, int jointType,
                             ros::Time capture_time);

  k4a_result_t getBodyIndexMap(const k4abt::frame& body_frame, sensor_msgs::ImagePtr body_index_map_image);

  k4a_result_t renderBodyIndexMapToROS(sensor_msgs::ImagePtr body_index_map_image, k4a::image& k4a_body_index_map,
                                       const k4abt::frame& body_frame);

  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
#endif

 private:
  k4a_result_t renderBGRA32ToROS(sensor_msgs::ImagePtr& rgb_frame, k4a::image& k4a_bgra_frame);
  k4a_result_t renderDepthToROS(sensor_msgs::ImagePtr& depth_image, k4a::image& k4a_depth_frame);
  k4a_result_t renderIrToROS(sensor_msgs::ImagePtr& ir_image, k4a::image& k4a_ir_frame);

  k4a_result_t fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud);
  k4a_result_t fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image,
                                   sensor_msgs::PointCloud2Ptr& point_cloud);

  void framePublisherThread();
#if defined(K4A_BODY_TRACKING)
  void bodyPublisherThread();
#endif
  void imuPublisherThread();

  // Gets a timestap from one of the captures images
  std::chrono::microseconds getCaptureTimestamp(const k4a::capture& capture);

  // Converts a k4a_image_t timestamp to a ros::Time object
  ros::Time timestampToROS(const std::chrono::microseconds& k4a_timestamp_us);

  // Converts a k4a_imu_sample_t timestamp to a ros::Time object
  ros::Time timestampToROS(const uint64_t& k4a_timestamp_us);

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

  // ROS Node variables
  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::NodeHandle node_rgb_;
  ros::NodeHandle node_ir_;

  image_transport::ImageTransport image_transport_;

  image_transport::Publisher rgb_raw_publisher_;
  ros::Publisher rgb_jpeg_publisher_;
  ros::Publisher rgb_raw_camerainfo_publisher_;

  image_transport::Publisher depth_raw_publisher_;
  ros::Publisher depth_raw_camerainfo_publisher_;

  image_transport::Publisher depth_rect_publisher_;
  ros::Publisher depth_rect_camerainfo_publisher_;

  image_transport::Publisher rgb_rect_publisher_;
  ros::Publisher rgb_rect_camerainfo_publisher_;

  image_transport::Publisher ir_raw_publisher_;
  ros::Publisher ir_raw_camerainfo_publisher_;

  ros::Publisher imu_orientation_publisher_;

  ros::Publisher pointcloud_publisher_;

  std::shared_ptr<camera_info_manager::CameraInfoManager> ci_mngr_rgb_, ci_mngr_ir_;

#if defined(K4A_BODY_TRACKING)
  ros::Publisher body_marker_publisher_;
  tf2_ros::TransformBroadcaster br;

  image_transport::Publisher body_index_map_publisher_;
  image_transport::Publisher image_tf_publisher_;
  image_transport::CameraSubscriber image_subscriber_;
  image_geometry::PinholeCameraModel cam_model_;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener* tfListener;
#endif

  // Parameters
  K4AROSDeviceParams params_;

  std::string serial_number_;

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
  
  std::vector<std::string> joint_names_{"Pelvis", "Spine_Naval", "Spine_Chest", "Neck", "Clavicle_left", "Shoulder_left", "Elbow_left", "Wrist_left", "Hand_left", "Handtip_left", "thumb_left", "Clavicle_right", "Shoulder_right", "Elbow_right", "Wrist_right", "Hand_right", "Handtip_right", "Thumb_right", "Hip_left", "Knee_left", "Ankle_left", "Foot_left", "Hip_right", "Knee_right", "Ankle_right", "Foot_right", "Head", "Nose", "Eye_Left", "Ear_Left", "Eye_Right", "Ear_Right"};
  size_t num_bodies;
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

void printTimestampDebugMessage(const std::string& name, const ros::Time& timestamp);

#endif  // K4A_ROS_DEVICE_H
