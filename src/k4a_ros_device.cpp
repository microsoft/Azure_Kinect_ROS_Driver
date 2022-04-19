// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

// System headers
//
#include <thread>

// Library headers
//
#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <k4a/k4a.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <k4a/k4a.hpp>
#include <unordered_map>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_types.h"

using namespace ros;
using namespace sensor_msgs;
using namespace image_transport;
using namespace std;

static const std::unordered_map<k4a_color_resolution_t, std::string> color_mode_string = {
  {K4A_COLOR_RESOLUTION_720P, "720P"},
  {K4A_COLOR_RESOLUTION_1080P, "1080P"},
  {K4A_COLOR_RESOLUTION_1440P, "1440P"},
  {K4A_COLOR_RESOLUTION_1536P, "1536P"},
  {K4A_COLOR_RESOLUTION_2160P, "2160P"},
  {K4A_COLOR_RESOLUTION_3072P, "3072P"},
};

static const std::unordered_map<k4a_depth_mode_t, std::string> depth_mode_string = {
  {K4A_DEPTH_MODE_NFOV_2X2BINNED, "NFOV_2X2BINNED"},
  {K4A_DEPTH_MODE_NFOV_UNBINNED, "NFOV_UNBINNED"},
  {K4A_DEPTH_MODE_WFOV_2X2BINNED, "WFOV_2X2BINNED"},
  {K4A_DEPTH_MODE_WFOV_UNBINNED, "WFOV_UNBINNED"},
  {K4A_DEPTH_MODE_PASSIVE_IR, "PASSIVE_IR"},
};

#if defined(K4A_BODY_TRACKING)
using namespace visualization_msgs;
#endif

K4AROSDevice::K4AROSDevice(const NodeHandle& n, const NodeHandle& p)
  : k4a_device_(nullptr),
    k4a_playback_handle_(nullptr),
// clang-format off
#if defined(K4A_BODY_TRACKING)
    k4abt_tracker_(nullptr),
    k4abt_tracker_queue_size_(0),
#endif
    // clang-format on
    node_(n),
    private_node_(p),
    node_rgb_("rgb"),
    node_ir_("ir"),
    image_transport_(n),
    last_capture_time_usec_(0),
    last_imu_time_usec_(0),
    imu_stream_end_of_file_(false)
{
  // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  private_node_.param(#param_variable, params_.param_variable, param_default_val);
  ROS_PARAM_LIST
#undef LIST_ENTRY

  if (!params_.recording_file.empty())
  {
    ROS_INFO("Node is started in playback mode");
    ROS_INFO_STREAM("Try to open recording file " << params_.recording_file);

    // Open recording file and print its length
    k4a_playback_handle_ = k4a::playback::open(params_.recording_file.c_str());
    auto recording_length = k4a_playback_handle_.get_recording_length();
    ROS_INFO_STREAM("Successfully openend recording file. Recording is " << recording_length.count() / 1000000
                                                                         << " seconds long");

    if (!k4a_playback_handle_.get_tag("K4A_DEVICE_SERIAL_NUMBER", &serial_number_))
    {
      serial_number_ = {};
      ROS_ERROR("Cannot read serial number from recording.");
    }

    // Get the recordings configuration to overwrite node parameters
    k4a_record_configuration_t record_config = k4a_playback_handle_.get_record_configuration();

    // Overwrite fps param with recording configuration for a correct loop rate in the frame publisher thread
    switch (record_config.camera_fps)
    {
      case K4A_FRAMES_PER_SECOND_5:
        params_.fps = 5;
        break;
      case K4A_FRAMES_PER_SECOND_15:
        params_.fps = 15;
        break;
      case K4A_FRAMES_PER_SECOND_30:
        params_.fps = 30;
        break;
      default:
        break;
    };

    // Disable color if the recording has no color track
    if (params_.color_enabled && !record_config.color_track_enabled)
    {
      ROS_WARN("Disabling color and rgb_point_cloud because recording has no color track");
      params_.color_enabled = false;
      params_.rgb_point_cloud = false;
    }
    // This is necessary because at the moment there are only checks in place which use BgraPixel size
    else if (params_.color_enabled && record_config.color_track_enabled)
    {
      if (params_.color_format == "jpeg" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_MJPG)
      {
        ROS_FATAL("Converting color images to K4A_IMAGE_FORMAT_COLOR_MJPG is not supported.");
        ros::requestShutdown();
        return;
      }
      if (params_.color_format == "bgra" && record_config.color_format != K4A_IMAGE_FORMAT_COLOR_BGRA32)
      {
        k4a_playback_handle_.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
      }
    }

    // Disable depth if the recording has neither ir track nor depth track
    if (!record_config.ir_track_enabled && !record_config.depth_track_enabled)
    {
      if (params_.depth_enabled)
      {
        ROS_WARN("Disabling depth because recording has neither ir track nor depth track");
        params_.depth_enabled = false;
      }
    }

    // Disable depth if the recording has no depth track
    if (!record_config.depth_track_enabled)
    {
      if (params_.point_cloud)
      {
        ROS_WARN("Disabling point cloud because recording has no depth track");
        params_.point_cloud = false;
      }
      if (params_.rgb_point_cloud)
      {
        ROS_WARN("Disabling rgb point cloud because recording has no depth track");
        params_.rgb_point_cloud = false;
      }
    }
  }
  else
  {
    // Print all parameters
    ROS_INFO("K4A Parameters:");
    params_.Print();

    // Setup the K4A device
    uint32_t k4a_device_count = k4a::device::get_installed_count();

    ROS_INFO_STREAM("Found " << k4a_device_count << " sensors");

    if (params_.sensor_sn != "")
    {
      ROS_INFO_STREAM("Searching for sensor with serial number: " << params_.sensor_sn);
    }
    else
    {
      ROS_INFO("No serial number provided: picking first sensor");
      ROS_WARN_COND(k4a_device_count > 1, "Multiple sensors connected! Picking first sensor.");
    }

    for (uint32_t i = 0; i < k4a_device_count; i++)
    {
      k4a::device device;
      try
      {
        device = k4a::device::open(i);
      }
      catch (exception)
      {
        ROS_ERROR_STREAM("Failed to open K4A device at index " << i);
        continue;
      }

      ROS_INFO_STREAM("K4A[" << i << "] : " << device.get_serialnum());

      // Try to match serial number
      if (params_.sensor_sn != "")
      {
        if (device.get_serialnum() == params_.sensor_sn)
        {
          k4a_device_ = std::move(device);
          break;
        }
      }
      // Pick the first device
      else if (i == 0)
      {
        k4a_device_ = std::move(device);
        break;
      }
    }

    if (!k4a_device_)
    {
      ROS_ERROR("Failed to open a K4A device. Cannot continue.");
      return;
    }

    serial_number_ = k4a_device_.get_serialnum();

    ROS_INFO_STREAM("K4A Serial Number: " << serial_number_);

    k4a_hardware_version_t version_info = k4a_device_.get_version();

    ROS_INFO("RGB Version: %d.%d.%d", version_info.rgb.major, version_info.rgb.minor, version_info.rgb.iteration);

    ROS_INFO("Depth Version: %d.%d.%d", version_info.depth.major, version_info.depth.minor,
             version_info.depth.iteration);

    ROS_INFO("Audio Version: %d.%d.%d", version_info.audio.major, version_info.audio.minor,
             version_info.audio.iteration);

    ROS_INFO("Depth Sensor Version: %d.%d.%d", version_info.depth_sensor.major, version_info.depth_sensor.minor,
             version_info.depth_sensor.iteration);
  }

  // Register our topics
  if (params_.color_format == "jpeg")
  {
    // JPEG images are directly published on 'rgb/image_raw/compressed' so that
    // others can subscribe to 'rgb/image_raw' with compressed_image_transport.
    // This technique is described in:
    // http://wiki.ros.org/compressed_image_transport#Publishing_compressed_images_directly
    rgb_jpeg_publisher_ = node_.advertise<CompressedImage>(node_.resolveName("rgb/image_raw") + "/compressed", 1);
  }
  else if (params_.color_format == "bgra")
  {
    rgb_raw_publisher_ = image_transport_.advertise("rgb/image_raw", 1);
  }
  rgb_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("rgb/camera_info", 1);

  static const std::string depth_raw_topic = "depth/image_raw";
  static const std::string depth_rect_topic = "depth_to_rgb/image_raw";
  if (params_.depth_unit == sensor_msgs::image_encodings::TYPE_16UC1) {
    // set lowest PNG compression for maximum FPS
    node_.setParam(node_.resolveName(depth_raw_topic) + "/compressed/format", "png");
    node_.setParam(node_.resolveName(depth_raw_topic) + "/compressed/png_level", 1);
    node_.setParam(node_.resolveName(depth_rect_topic) + "/compressed/format", "png");
    node_.setParam(node_.resolveName(depth_rect_topic) + "/compressed/png_level", 1);
  }

  depth_raw_publisher_ = image_transport_.advertise(depth_raw_topic, 1);
  depth_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("depth/camera_info", 1);

  depth_rect_publisher_ = image_transport_.advertise(depth_rect_topic, 1);
  depth_rect_camerainfo_publisher_ = node_.advertise<CameraInfo>("depth_to_rgb/camera_info", 1);

  rgb_rect_publisher_ = image_transport_.advertise("rgb_to_depth/image_raw", 1);
  rgb_rect_camerainfo_publisher_ = node_.advertise<CameraInfo>("rgb_to_depth/camera_info", 1);

  ir_raw_publisher_ = image_transport_.advertise("ir/image_raw", 1);
  ir_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("ir/camera_info", 1);

  imu_orientation_publisher_ = node_.advertise<Imu>("imu", 200);

  if (params_.point_cloud || params_.rgb_point_cloud) {
    pointcloud_publisher_ = node_.advertise<PointCloud2>("points2", 1);
  }

  if (k4a_playback_handle_) {
    // override color and depth mode configuration with settings from log file
    const k4a_color_resolution_t cm = k4a_playback_handle_.get_record_configuration().color_resolution;
    if (cm != K4A_COLOR_RESOLUTION_OFF) {
      params_.color_resolution = color_mode_string.at(cm);
    }

    const k4a_depth_mode_t dm = k4a_playback_handle_.get_record_configuration().depth_mode;
    if (dm != K4A_DEPTH_MODE_OFF) {
      params_.depth_mode = depth_mode_string.at(dm);
    }
  }

  // load calibration file from provided path or use default camera calibration URL at $HOME/.ros/camera_info/<cname>.yaml
  const std::string calibration_file_name_rgb = "azure_kinect_rgb_"+serial_number_+"_"+params_.color_resolution;
  const std::string calibration_file_name_ir = "azure_kinect_ir_"+serial_number_+"_"+params_.depth_mode;
  const std::string calibration_url_rgb = params_.calibration_url.empty() ? std::string{} : params_.calibration_url + '/' + calibration_file_name_rgb + ".yaml";
  const std::string calibration_url_ir = params_.calibration_url.empty() ? std::string{} : params_.calibration_url + '/' + calibration_file_name_ir + ".yaml";

  ci_mngr_rgb_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_rgb_, calibration_file_name_rgb, calibration_url_rgb);
  ci_mngr_ir_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_ir_, calibration_file_name_ir, calibration_url_ir);

#if defined(K4A_BODY_TRACKING)
  if (params_.body_tracking_enabled) {    
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    body_marker_publisher_ = node_.advertise<MarkerArray>("body_tracking_data", 1);

    body_index_map_publisher_ = image_transport_.advertise("body_index_map/image_raw", 1);

    image_subscriber_ = image_transport_.subscribeCamera("rgb/image_raw", 1, &K4AROSDevice::imageCallback, this);
    image_tf_publisher_ = image_transport_.advertise("image_tf", 1); 
  
  }
#endif
}

K4AROSDevice::~K4AROSDevice()
{
  // Start tearing down the publisher threads
  running_ = false;

#if defined(K4A_BODY_TRACKING)
  // Join the publisher thread
  ROS_INFO("Joining body publisher thread");
  body_publisher_thread_.join();
  ROS_INFO("Body publisher thread joined");
#endif

  // Join the publisher thread
  ROS_INFO("Joining camera publisher thread");
  frame_publisher_thread_.join();
  ROS_INFO("Camera publisher thread joined");

  // Join the publisher thread
  ROS_INFO("Joining IMU publisher thread");
  imu_publisher_thread_.join();
  ROS_INFO("IMU publisher thread joined");

  stopCameras();
  stopImu();

  if (k4a_playback_handle_)
  {
    k4a_playback_handle_.close();
  }

#if defined(K4A_BODY_TRACKING)
  if (k4abt_tracker_)
  {
    k4abt_tracker_.shutdown();
  }
#endif
}

k4a_result_t K4AROSDevice::startCameras()
{
  k4a_device_configuration_t k4a_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

  if (k4a_device_)
  {
    k4a_result_t result = params_.GetDeviceConfig(&k4a_configuration);
    if (result != K4A_RESULT_SUCCEEDED)
    {
      ROS_ERROR("Failed to generate a device configuration. Not starting camera!");
      return result;
    }

    // Now that we have a proposed camera configuration, we can
    // initialize the class which will take care of device calibration information
    calibration_data_.initialize(k4a_device_, k4a_configuration.depth_mode, k4a_configuration.color_resolution,
                                 params_);
  }
  else if (k4a_playback_handle_)
  {
    // initialize the class which will take care of device calibration information from the playback_handle
    calibration_data_.initialize(k4a_playback_handle_, params_);
  }

#if defined(K4A_BODY_TRACKING)
  // When calibration is initialized the body tracker can be created with the device calibration
  if (params_.body_tracking_enabled)
  {
    k4abt_tracker_ = k4abt::tracker::create(calibration_data_.k4a_calibration_);
    k4abt_tracker_.set_temporal_smoothing(params_.body_tracking_smoothing_factor);
  }
#endif

  if (k4a_device_)
  {
    ROS_INFO_STREAM("STARTING CAMERAS");
    k4a_device_.start_cameras(&k4a_configuration);
  }

  // Cannot assume the device timestamp begins increasing upon starting the cameras.
  // If we set the time base here, depending on the machine performance, the new timestamp
  // would lag the value of ros::Time::now() by at least 0.5 secs which is much larger than
  // the real transmission delay as can be observed using the rqt_plot tool.
  // start_time_ = ros::Time::now();

  // Prevent the worker thread from exiting immediately
  running_ = true;

  // Start the thread that will poll the cameras and publish frames
  frame_publisher_thread_ = thread(&K4AROSDevice::framePublisherThread, this);
#if defined(K4A_BODY_TRACKING)
  body_publisher_thread_ = thread(&K4AROSDevice::bodyPublisherThread, this);
#endif

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::startImu()
{
  if (k4a_device_)
  {
    ROS_INFO_STREAM("STARTING IMU");
    k4a_device_.start_imu();
  }

  // Start the IMU publisher thread
  imu_publisher_thread_ = thread(&K4AROSDevice::imuPublisherThread, this);

  return K4A_RESULT_SUCCEEDED;
}

void K4AROSDevice::stopCameras()
{
  if (k4a_device_)
  {
    // Stop the K4A SDK
    ROS_INFO("Stopping K4A device");
    k4a_device_.stop_cameras();
    ROS_INFO("K4A device stopped");
  }
}

void K4AROSDevice::stopImu()
{
  if (k4a_device_)
  {
    k4a_device_.stop_imu();
  }
}

k4a_result_t K4AROSDevice::getDepthFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& depth_image,
                                         bool rectified = false)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render depth frame: no frame");
    return K4A_RESULT_FAILED;
  }

  if (rectified)
  {
    calibration_data_.k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
                                                                      &calibration_data_.transformed_depth_image_);

    return renderDepthToROS(depth_image, calibration_data_.transformed_depth_image_);
  }

  return renderDepthToROS(depth_image, k4a_depth_frame);
}

k4a_result_t K4AROSDevice::renderDepthToROS(sensor_msgs::ImagePtr& depth_image, k4a::image& k4a_depth_frame)
{
  cv::Mat depth_frame_buffer_mat(k4a_depth_frame.get_height_pixels(), k4a_depth_frame.get_width_pixels(), CV_16UC1,
                                 k4a_depth_frame.get_buffer());
  std::string encoding;

  if (params_.depth_unit == sensor_msgs::image_encodings::TYPE_32FC1) {
    // convert from 16 bit integer millimetre to 32 bit float metre
    depth_frame_buffer_mat.convertTo(depth_frame_buffer_mat, CV_32FC1, 1.0 / 1000.0f);
    encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  }
  else if (params_.depth_unit == sensor_msgs::image_encodings::TYPE_16UC1) {
    // source data is already in 'K4A_IMAGE_FORMAT_DEPTH16' format
    encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  }
  else {
    ROS_ERROR_STREAM("Invalid depth unit: " << params_.depth_unit);
    return K4A_RESULT_FAILED;
  }

  depth_image =
      cv_bridge::CvImage(std_msgs::Header(), encoding, depth_frame_buffer_mat).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getIrFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& ir_image)
{
  k4a::image k4a_ir_frame = capture.get_ir_image();

  if (!k4a_ir_frame)
  {
    ROS_ERROR("Cannot render IR frame: no frame");
    return K4A_RESULT_FAILED;
  }

  return renderIrToROS(ir_image, k4a_ir_frame);
}

k4a_result_t K4AROSDevice::renderIrToROS(sensor_msgs::ImagePtr& ir_image, k4a::image& k4a_ir_frame)
{
  cv::Mat ir_buffer_mat(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_16UC1,
                        k4a_ir_frame.get_buffer());

  // Rescale the image to mono8 for visualization and usage for visual(-inertial) odometry.
  if (params_.rescale_ir_to_mono8)
  {
    cv::Mat new_image(k4a_ir_frame.get_height_pixels(), k4a_ir_frame.get_width_pixels(), CV_8UC1);
    // Use a scaling factor to re-scale the image. If using the illuminators, a value of 1 is appropriate.
    // If using PASSIVE_IR, then a value of 10 is more appropriate; k4aviewer does a similar conversion.
    ir_buffer_mat.convertTo(new_image, CV_8UC1, params_.ir_mono8_scaling_factor);
    ir_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, new_image).toImageMsg();
  }
  else
  {
    ir_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getJpegRgbFrame(const k4a::capture& capture, sensor_msgs::CompressedImagePtr& jpeg_image)
{
  k4a::image k4a_jpeg_frame = capture.get_color_image();

  if (!k4a_jpeg_frame)
  {
    ROS_ERROR("Cannot render Jpeg frame: no frame");
    return K4A_RESULT_FAILED;
  }

  const uint8_t* jpeg_frame_buffer = k4a_jpeg_frame.get_buffer();
  jpeg_image->format = "bgra8; jpeg compressed bgr8";
  jpeg_image->data.assign(jpeg_frame_buffer, jpeg_frame_buffer + k4a_jpeg_frame.get_size());
  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getRbgFrame(const k4a::capture& capture, sensor_msgs::ImagePtr& rgb_image,
                                       bool rectified = false)
{
  k4a::image k4a_bgra_frame = capture.get_color_image();

  if (!k4a_bgra_frame)
  {
    ROS_ERROR("Cannot render BGRA frame: no frame");
    return K4A_RESULT_FAILED;
  }

  size_t color_image_size =
      static_cast<size_t>(k4a_bgra_frame.get_width_pixels() * k4a_bgra_frame.get_height_pixels()) * sizeof(BgraPixel);

  if (k4a_bgra_frame.get_size() != color_image_size)
  {
    ROS_WARN("Invalid k4a_bgra_frame returned from K4A");
    return K4A_RESULT_FAILED;
  }

  if (rectified)
  {
    k4a::image k4a_depth_frame = capture.get_depth_image();

    calibration_data_.k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame, k4a_bgra_frame,
                                                                      &calibration_data_.transformed_rgb_image_);

    return renderBGRA32ToROS(rgb_image, calibration_data_.transformed_rgb_image_);
  }

  return renderBGRA32ToROS(rgb_image, k4a_bgra_frame);
}

// Helper function that renders any BGRA K4A frame to a ROS ImagePtr. Useful for rendering intermediary frames
// during debugging of image processing functions
k4a_result_t K4AROSDevice::renderBGRA32ToROS(sensor_msgs::ImagePtr& rgb_image, k4a::image& k4a_bgra_frame)
{
  cv::Mat rgb_buffer_mat(k4a_bgra_frame.get_height_pixels(), k4a_bgra_frame.get_width_pixels(), CV_8UC4,
                         k4a_bgra_frame.get_buffer());

  rgb_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGRA8, rgb_buffer_mat).toImageMsg();

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getRgbPointCloudInDepthFrame(const k4a::capture& capture,
                                                        sensor_msgs::PointCloud2Ptr& point_cloud)
{
  const k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  const k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // Transform color image into the depth camera frame:
  calibration_data_.k4a_transformation_.color_image_to_depth_camera(k4a_depth_frame, k4a_bgra_frame,
                                                                    &calibration_data_.transformed_rgb_image_);

  // Tranform depth image to point cloud
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_.point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());

  return fillColorPointCloud(calibration_data_.point_cloud_image_, calibration_data_.transformed_rgb_image_,
                             point_cloud);
}

k4a_result_t K4AROSDevice::getRgbPointCloudInRgbFrame(const k4a::capture& capture,
                                                      sensor_msgs::PointCloud2Ptr& point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();
  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  k4a::image k4a_bgra_frame = capture.get_color_image();
  if (!k4a_bgra_frame)
  {
    ROS_ERROR("Cannot render RGB point cloud: no BGRA frame");
    return K4A_RESULT_FAILED;
  }

  // transform depth image into color camera geometry
  calibration_data_.k4a_transformation_.depth_image_to_color_camera(k4a_depth_frame,
                                                                    &calibration_data_.transformed_depth_image_);

  // Tranform depth image to point cloud (note that this is now from the perspective of the color camera)
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(
      calibration_data_.transformed_depth_image_, K4A_CALIBRATION_TYPE_COLOR, &calibration_data_.point_cloud_image_);

  point_cloud->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());

  return fillColorPointCloud(calibration_data_.point_cloud_image_, k4a_bgra_frame, point_cloud);
}

k4a_result_t K4AROSDevice::getPointCloud(const k4a::capture& capture, sensor_msgs::PointCloud2Ptr& point_cloud)
{
  k4a::image k4a_depth_frame = capture.get_depth_image();

  if (!k4a_depth_frame)
  {
    ROS_ERROR("Cannot render point cloud: no depth frame");
    return K4A_RESULT_FAILED;
  }

  point_cloud->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());

  // Tranform depth image to point cloud
  calibration_data_.k4a_transformation_.depth_image_to_point_cloud(k4a_depth_frame, K4A_CALIBRATION_TYPE_DEPTH,
                                                                   &calibration_data_.point_cloud_image_);

  return fillPointCloud(calibration_data_.point_cloud_image_, point_cloud);
}

k4a_result_t K4AROSDevice::fillColorPointCloud(const k4a::image& pointcloud_image, const k4a::image& color_image,
                                               sensor_msgs::PointCloud2Ptr& point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();
  const size_t pixel_count = color_image.get_size() / sizeof(BgraPixel);
  if (point_count != pixel_count)
  {
    ROS_WARN("Color and depth image sizes do not match!");
    return K4A_RESULT_FAILED;
  }

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud, "b");

  pcd_modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());
  const uint8_t* color_buffer = color_image.get_buffer();

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    // Z in image frame:
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);
    // Alpha value:
    uint8_t a = color_buffer[4 * i + 3];
    if (z <= 0.0f || a == 0)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      *iter_r = *iter_g = *iter_b = 0;
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;

      *iter_r = color_buffer[4 * i + 2];
      *iter_g = color_buffer[4 * i + 1];
      *iter_b = color_buffer[4 * i + 0];
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::fillPointCloud(const k4a::image& pointcloud_image, sensor_msgs::PointCloud2Ptr& point_cloud)
{
  point_cloud->height = pointcloud_image.get_height_pixels();
  point_cloud->width = pointcloud_image.get_width_pixels();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;

  const size_t point_count = pointcloud_image.get_height_pixels() * pointcloud_image.get_width_pixels();

  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  pcd_modifier.resize(point_count);

  const int16_t* point_cloud_buffer = reinterpret_cast<const int16_t*>(pointcloud_image.get_buffer());

  for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
  {
    float z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

    if (z <= 0.0f)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 0]);
      *iter_y = kMillimeterToMeter * static_cast<float>(point_cloud_buffer[3 * i + 1]);
      *iter_z = kMillimeterToMeter * z;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}

k4a_result_t K4AROSDevice::getImuFrame(const k4a_imu_sample_t& sample, sensor_msgs::ImuPtr& imu_msg)
{
  imu_msg->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.imu_frame_;
  imu_msg->header.stamp = timestampToROS(sample.acc_timestamp_usec);

  // The correct convention in ROS is to publish the raw sensor data, in the
  // sensor coordinate frame. Do that here.
  imu_msg->angular_velocity.x = sample.gyro_sample.xyz.x;
  imu_msg->angular_velocity.y = sample.gyro_sample.xyz.y;
  imu_msg->angular_velocity.z = sample.gyro_sample.xyz.z;

  imu_msg->linear_acceleration.x = sample.acc_sample.xyz.x;
  imu_msg->linear_acceleration.y = sample.acc_sample.xyz.y;
  imu_msg->linear_acceleration.z = sample.acc_sample.xyz.z;

  // Disable the orientation component of the IMU message since it's invalid
  imu_msg->orientation_covariance[0] = -1.0;

  return K4A_RESULT_SUCCEEDED;
}

#if defined(K4A_BODY_TRACKING)
k4a_result_t K4AROSDevice::getBodyMarker(const k4abt_body_t& body, MarkerPtr marker_msg, geometry_msgs::TransformStamped& transform_msg, int bodyNum, int jointType,
                                         ros::Time capture_time)
{
  k4a_float3_t position = body.skeleton.joints[jointType].position;
  k4a_quaternion_t orientation = body.skeleton.joints[jointType].orientation;
  std::string depth_frame = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  std::string rgb_frame = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;

  marker_msg->header.frame_id = depth_frame;
  marker_msg->header.stamp = capture_time;

  // Set the lifetime to 0.25 to prevent flickering for even 5fps configurations.
  // New markers with the same ID will replace old markers as soon as they arrive.
  marker_msg->lifetime = ros::Duration(0.25);
  marker_msg->id = body.id * 100 + jointType;
  marker_msg->type = Marker::SPHERE;

  Color color = BODY_COLOR_PALETTE[body.id % BODY_COLOR_PALETTE.size()];

  marker_msg->color.a = color.a;
  marker_msg->color.r = color.r;
  marker_msg->color.g = color.g;
  marker_msg->color.b = color.b;

  marker_msg->scale.x = 0.05;
  marker_msg->scale.y = 0.05;
  marker_msg->scale.z = 0.05;

  marker_msg->pose.position.x = position.v[0] / 1000.0f;
  marker_msg->pose.position.y = position.v[1] / 1000.0f;
  marker_msg->pose.position.z = position.v[2] / 1000.0f;
  marker_msg->pose.orientation.w = orientation.wxyz.w;
  marker_msg->pose.orientation.x = orientation.wxyz.x;
  marker_msg->pose.orientation.y = orientation.wxyz.y;
  marker_msg->pose.orientation.z = orientation.wxyz.z;

 //try transforming from depth_camera_link to rgb_camera_link by waiting for the transform upto 1 second
  geometry_msgs::TransformStamped depth_link_to_rgb_link;
  try{
    depth_link_to_rgb_link = tfBuffer.lookupTransform(rgb_frame , depth_frame,
                              ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  //Pose msg which is used to transform the pose to the rgb camera link
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = position.v[0] / 1000.0f;
  pose_msg.position.y = position.v[1] / 1000.0f;
  pose_msg.position.z = position.v[2] / 1000.0f;
  pose_msg.orientation.w = orientation.wxyz.w;
  pose_msg.orientation.x = orientation.wxyz.x;
  pose_msg.orientation.y = orientation.wxyz.y;
  pose_msg.orientation.z = orientation.wxyz.z;


  tf2::doTransform(pose_msg, pose_msg, depth_link_to_rgb_link);

  transform_msg.header.stamp = capture_time;
  transform_msg.header.frame_id = rgb_frame;
  transform_msg.child_frame_id = joint_names_[jointType] + std::to_string(bodyNum);

  transform_msg.transform.translation.x = pose_msg.position.x;
  transform_msg.transform.translation.y = pose_msg.position.y;
  transform_msg.transform.translation.z = pose_msg.position.z;

  transform_msg.transform.rotation.w = pose_msg.orientation.w;
  transform_msg.transform.rotation.x = pose_msg.orientation.x;
  transform_msg.transform.rotation.y = pose_msg.orientation.y;
  transform_msg.transform.rotation.z = pose_msg.orientation.z;

  return K4A_RESULT_SUCCEEDED;
}

//method to project a tf frame to an image
void K4AROSDevice::imageCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv::Mat image;
  cv_bridge::CvImagePtr input_bridge;
  try {
    input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image = input_bridge->image;
  }
  catch (cv_bridge::Exception& ex){
    ROS_ERROR("[draw_frames] Failed to convert image");
    return;
  }

  cam_model_.fromCameraInfo(info_msg);

  std::vector<std::string> frame_ids_;
  for(int i = 0; i < num_bodies; ++i){
      std::transform(joint_names_.begin(), joint_names_.end(), back_inserter(frame_ids_), [&i](std::string j){return j + std::to_string(i);});    
  }

  for(const std::string frame_id: frame_ids_) {
    
    geometry_msgs::TransformStamped transform_msg;
    try{

      transform_msg = tfBuffer.lookupTransform(cam_model_.tfFrame(), frame_id,
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("Unable to look up the transform between the frames, %s",ex.what());
      return;
    }

    cv::Point3d pt_cv(transform_msg.transform.translation.x, transform_msg.transform.translation.y, transform_msg.transform.translation.z);
    cv::Point2d uv;
    uv = cam_model_.project3dToPixel(pt_cv);

    static const int RADIUS = 10;
    cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
  }

  image_tf_publisher_.publish(input_bridge->toImageMsg());
}

k4a_result_t K4AROSDevice::getBodyIndexMap(const k4abt::frame& body_frame, sensor_msgs::ImagePtr body_index_map_image)
{
  k4a::image k4a_body_index_map = body_frame.get_body_index_map();

  if (!k4a_body_index_map)
  {
    ROS_ERROR("Cannot render body index map: no body index map");
    return K4A_RESULT_FAILED;
  }

  return renderBodyIndexMapToROS(body_index_map_image, k4a_body_index_map, body_frame);
}

k4a_result_t K4AROSDevice::renderBodyIndexMapToROS(sensor_msgs::ImagePtr body_index_map_image,
                                                   k4a::image& k4a_body_index_map, const k4abt::frame& body_frame)
{
  // Access the body index map as an array of uint8 pixels
  BodyIndexMapPixel* body_index_map_frame_buffer = k4a_body_index_map.get_buffer();
  auto body_index_map_pixel_count = k4a_body_index_map.get_size() / sizeof(BodyIndexMapPixel);

  // Build the ROS message
  body_index_map_image->height = k4a_body_index_map.get_height_pixels();
  body_index_map_image->width = k4a_body_index_map.get_width_pixels();
  body_index_map_image->encoding = sensor_msgs::image_encodings::MONO8;
  body_index_map_image->is_bigendian = false;
  body_index_map_image->step = k4a_body_index_map.get_width_pixels() * sizeof(BodyIndexMapPixel);

  // Enlarge the data buffer in the ROS message to hold the frame
  body_index_map_image->data.resize(body_index_map_image->height * body_index_map_image->step);

  // If the pixel doesn't belong to a detected body the pixels value will be 255 (K4ABT_BODY_INDEX_MAP_BACKGROUND).
  // If the pixel belongs to a detected body the value is calculated by body id mod 255.
  // This means that up to body id 254 the value is equals the body id.
  // Afterwards it will lose the relation to the body id and is only a information for the segmentation of the image.
  for (size_t i = 0; i < body_index_map_pixel_count; ++i)
  {
    BodyIndexMapPixel val = body_index_map_frame_buffer[i];
    if (val == K4ABT_BODY_INDEX_MAP_BACKGROUND)
    {
      body_index_map_image->data[i] = K4ABT_BODY_INDEX_MAP_BACKGROUND;
    }
    else
    {
      auto body_id = k4abt_frame_get_body_id(body_frame.handle(), val);
      body_index_map_image->data[i] = body_id % K4ABT_BODY_INDEX_MAP_BACKGROUND;
    }
  }

  return K4A_RESULT_SUCCEEDED;
}
#endif

void K4AROSDevice::framePublisherThread()
{
  ros::Rate loop_rate(params_.fps);

  k4a_wait_result_t wait_result;
  k4a_result_t result;

  CameraInfo rgb_raw_camera_info;
  CameraInfo depth_raw_camera_info;
  CameraInfo rgb_rect_camera_info;
  CameraInfo depth_rect_camera_info;
  CameraInfo ir_raw_camera_info;

  Time capture_time;

  k4a::capture capture;

  if (ci_mngr_rgb_->isCalibrated())
  {
    rgb_raw_camera_info = depth_rect_camera_info = ci_mngr_rgb_->getCameraInfo();
    rgb_raw_camera_info.header.frame_id = depth_rect_camera_info.header.frame_id = \
        calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
  }
  else
  {
    calibration_data_.getRgbCameraInfo(rgb_raw_camera_info);
    calibration_data_.getRgbCameraInfo(depth_rect_camera_info);
  }

  if (ci_mngr_ir_->isCalibrated())
  {
    depth_raw_camera_info = rgb_rect_camera_info = ir_raw_camera_info = ci_mngr_ir_->getCameraInfo();
    depth_raw_camera_info.header.frame_id = rgb_rect_camera_info.header.frame_id = ir_raw_camera_info.header.frame_id = \
        calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  }
  else
  {
    calibration_data_.getDepthCameraInfo(depth_raw_camera_info);
    calibration_data_.getDepthCameraInfo(rgb_rect_camera_info);
    calibration_data_.getDepthCameraInfo(ir_raw_camera_info);
  }

  //First frame needs longer to arrive, we wait up to 4 seconds for it
  const std::chrono::milliseconds firstFrameWaitTime = std::chrono::milliseconds(4 * 1000);
  //fail if we did non receive 5 consecutive frames in a row
  const std::chrono::milliseconds regularFrameWaitTime = std::chrono::milliseconds(1000 * 5 / params_.fps);
  std::chrono::milliseconds waitTime = firstFrameWaitTime;

  while (running_ && ros::ok() && !ros::isShuttingDown())
  {
    if (k4a_device_)
    {
      if (!k4a_device_.get_capture(&capture, waitTime))
      {
        ROS_FATAL("Failed to poll cameras: node cannot continue.");
        ros::requestShutdown();
        return;
      }
      else
      {
        if (params_.depth_enabled)
        {
          // Update the timestamp offset based on the difference between the system timestamp (i.e., arrival at USB bus)
          // and device timestamp (i.e., hardware clock at exposure start).
          updateTimestampOffset(capture.get_ir_image().get_device_timestamp(),
                                capture.get_ir_image().get_system_timestamp());
        }
        else if (params_.color_enabled)
        {
          updateTimestampOffset(capture.get_color_image().get_device_timestamp(),
                                capture.get_color_image().get_system_timestamp());
        }
      }
      waitTime = regularFrameWaitTime;
    }
    else if (k4a_playback_handle_)
    {
      std::lock_guard<std::mutex> guard(k4a_playback_handle_mutex_);
      if (!k4a_playback_handle_.get_next_capture(&capture))
      {
        // rewind recording if looping is enabled
        if (params_.recording_loop_enabled)
        {
          k4a_playback_handle_.seek_timestamp(std::chrono::microseconds(0), K4A_PLAYBACK_SEEK_BEGIN);
          k4a_playback_handle_.get_next_capture(&capture);
          imu_stream_end_of_file_ = false;
          last_imu_time_usec_ = 0;
        }
        else
        {
          ROS_INFO("Recording reached end of file. node cannot continue.");
          ros::requestShutdown();
          return;
        }
      }

      last_capture_time_usec_ = getCaptureTimestamp(capture).count();
    }

    CompressedImagePtr rgb_jpeg_frame(new CompressedImage);
    ImagePtr rgb_raw_frame(new Image);
    ImagePtr rgb_rect_frame(new Image);
    ImagePtr depth_raw_frame(new Image);
    ImagePtr depth_rect_frame(new Image);
    ImagePtr ir_raw_frame(new Image);
    PointCloud2Ptr point_cloud(new PointCloud2);

    if (params_.depth_enabled)
    {
      // Only do compute if we have subscribers
      // Only create ir frame when we are using a device or we have an ir image.
      // Recordings may not have synchronized captures. For unsynchronized captures without ir image skip ir frame.
      if ((ir_raw_publisher_.getNumSubscribers() > 0 || ir_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
          (k4a_device_ || capture.get_ir_image() != nullptr))
      {
        // IR images are available in all depth modes
        result = getIrFrame(capture, ir_raw_frame);

        if (result != K4A_RESULT_SUCCEEDED)
        {
          ROS_ERROR_STREAM("Failed to get raw IR frame");
          ros::shutdown();
          return;
        }
        else if (result == K4A_RESULT_SUCCEEDED)
        {
          capture_time = timestampToROS(capture.get_ir_image().get_device_timestamp());

          // Re-sychronize the timestamps with the capture timestamp
          ir_raw_camera_info.header.stamp = capture_time;
          ir_raw_frame->header.stamp = capture_time;
          ir_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;

          ir_raw_publisher_.publish(ir_raw_frame);
          ir_raw_camerainfo_publisher_.publish(ir_raw_camera_info);
        }
      }

      // Depth images are not available in PASSIVE_IR mode
      if (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR)
      {
        // Only create depth frame when we are using a device or we have an depth image.
        // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip depth
        // frame.
        if ((depth_raw_publisher_.getNumSubscribers() > 0 || depth_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
            (k4a_device_ || capture.get_depth_image() != nullptr))
        {
          result = getDepthFrame(capture, depth_raw_frame);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get raw depth frame");
            ros::shutdown();
            return;
          }
          else if (result == K4A_RESULT_SUCCEEDED)
          {
            capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());

            // Re-sychronize the timestamps with the capture timestamp
            depth_raw_camera_info.header.stamp = capture_time;
            depth_raw_frame->header.stamp = capture_time;
            depth_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;

            depth_raw_publisher_.publish(depth_raw_frame);
            depth_raw_camerainfo_publisher_.publish(depth_raw_camera_info);
          }
        }

        // We can only rectify the depth into the color co-ordinates if the color camera is enabled!
        // Only create rect depth frame when we are using a device or we have an depth image.
        // Recordings may not have synchronized captures. For unsynchronized captures without depth image skip rect
        // depth frame.
        if (params_.color_enabled &&
            (depth_rect_publisher_.getNumSubscribers() > 0 ||
             depth_rect_camerainfo_publisher_.getNumSubscribers() > 0) &&
            (k4a_device_ || capture.get_depth_image() != nullptr))
        {
          result = getDepthFrame(capture, depth_rect_frame, true /* rectified */);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get rectifed depth frame");
            ros::shutdown();
            return;
          }
          else if (result == K4A_RESULT_SUCCEEDED)
          {
            capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());

            depth_rect_frame->header.stamp = capture_time;
            depth_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
            depth_rect_publisher_.publish(depth_rect_frame);

            // Re-synchronize the header timestamps since we cache the camera calibration message
            depth_rect_camera_info.header.stamp = capture_time;
            depth_rect_camerainfo_publisher_.publish(depth_rect_camera_info);
          }
        }

#if defined(K4A_BODY_TRACKING)
        // Publish body markers when body tracking is enabled and a depth image is available
        if (params_.body_tracking_enabled &&  k4abt_tracker_queue_size_ < 3 &&
            (body_marker_publisher_.getNumSubscribers() > 0 || body_index_map_publisher_.getNumSubscribers() > 0))
        {
          if (!k4abt_tracker_.enqueue_capture(capture))
          {
            ROS_ERROR("Error! Add capture to tracker process queue failed!");
            ros::shutdown();
            return;
          }
          else
          {
            ++k4abt_tracker_queue_size_;
          }
        }
#endif
      }
    }

    if (params_.color_enabled)
    {
      // Only create rgb frame when we are using a device or we have a color image.
      // Recordings may not have synchronized captures. For unsynchronized captures without color image skip rgb frame.
      if (params_.color_format == "jpeg")
      {
        if ((rgb_jpeg_publisher_.getNumSubscribers() > 0 || rgb_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
            (k4a_device_ || capture.get_color_image() != nullptr))
        {
          result = getJpegRgbFrame(capture, rgb_jpeg_frame);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get Jpeg frame");
            ros::shutdown();
            return;
          }

          capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());

          rgb_jpeg_frame->header.stamp = capture_time;
          rgb_jpeg_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
          rgb_jpeg_publisher_.publish(rgb_jpeg_frame);

          // Re-synchronize the header timestamps since we cache the camera calibration message
          rgb_raw_camera_info.header.stamp = capture_time;
          rgb_raw_camerainfo_publisher_.publish(rgb_raw_camera_info);
        }
      }
      else if (params_.color_format == "bgra")
      {
        if ((rgb_raw_publisher_.getNumSubscribers() > 0 || rgb_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
            (k4a_device_ || capture.get_color_image() != nullptr))
        {
          result = getRbgFrame(capture, rgb_raw_frame);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get RGB frame");
            ros::shutdown();
            return;
          }

          capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());

          rgb_raw_frame->header.stamp = capture_time;
          rgb_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
          rgb_raw_publisher_.publish(rgb_raw_frame);

          // Re-synchronize the header timestamps since we cache the camera calibration message
          rgb_raw_camera_info.header.stamp = capture_time;
          rgb_raw_camerainfo_publisher_.publish(rgb_raw_camera_info);
        }

        // We can only rectify the color into the depth co-ordinates if the depth camera is enabled and processing depth
        // data Only create rgb rect frame when we are using a device or we have a synchronized image. Recordings may
        // not have synchronized captures. For unsynchronized captures image skip rgb rect frame.
        if (params_.depth_enabled && (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR) &&
            (rgb_rect_publisher_.getNumSubscribers() > 0 || rgb_rect_camerainfo_publisher_.getNumSubscribers() > 0) &&
            (k4a_device_ || (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
        {
          result = getRbgFrame(capture, rgb_rect_frame, true /* rectified */);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get rectifed depth frame");
            ros::shutdown();
            return;
          }

          capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());

          rgb_rect_frame->header.stamp = capture_time;
          rgb_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
          rgb_rect_publisher_.publish(rgb_rect_frame);

          // Re-synchronize the header timestamps since we cache the camera calibration message
          rgb_rect_camera_info.header.stamp = capture_time;
          rgb_rect_camerainfo_publisher_.publish(rgb_rect_camera_info);
        }
      }
    }

    // Only create pointcloud when we are using a device or we have a synchronized image.
    // Recordings may not have synchronized captures. In unsynchronized captures skip point cloud.
    if (pointcloud_publisher_.getNumSubscribers() > 0 &&
        (k4a_device_ || (capture.get_color_image() != nullptr && capture.get_depth_image() != nullptr)))
    {
      if (params_.rgb_point_cloud)
      {
        if (params_.point_cloud_in_depth_frame)
        {
          result = getRgbPointCloudInDepthFrame(capture, point_cloud);
        }
        else
        {
          result = getRgbPointCloudInRgbFrame(capture, point_cloud);
        }

        if (result != K4A_RESULT_SUCCEEDED)
        {
          ROS_ERROR_STREAM("Failed to get RGB Point Cloud");
          ros::shutdown();
          return;
        }
      }
      else if (params_.point_cloud)
      {
        result = getPointCloud(capture, point_cloud);

        if (result != K4A_RESULT_SUCCEEDED)
        {
          ROS_ERROR_STREAM("Failed to get Point Cloud");
          ros::shutdown();
          return;
        }
      }

      if (params_.point_cloud || params_.rgb_point_cloud)
      {
        pointcloud_publisher_.publish(point_cloud);
      }
    }

    if (loop_rate.cycleTime() > loop_rate.expectedCycleTime())
    {
      ROS_WARN_STREAM_THROTTLE(10, "Image processing thread is running behind."
                                       << std::endl
                                       << "Expected max loop time: " << loop_rate.expectedCycleTime() << std::endl
                                       << "Actual loop time: " << loop_rate.cycleTime() << std::endl);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

#if defined(K4A_BODY_TRACKING)
void K4AROSDevice::bodyPublisherThread()
{
  while (running_ && ros::ok() && !ros::isShuttingDown())
  {
    if (k4abt_tracker_queue_size_ > 0)
    {
      k4abt::frame body_frame = k4abt_tracker_.pop_result();
      --k4abt_tracker_queue_size_;

      if (body_frame == nullptr)
      {
        ROS_ERROR_STREAM("Pop body frame result failed!");
        ros::shutdown();
        return;
      }
      else
      {
        auto capture_time = timestampToROS(body_frame.get_device_timestamp());

        if (body_marker_publisher_.getNumSubscribers() > 0)
        {
          // Joint marker array
          
          MarkerArrayPtr markerArrayPtr(new MarkerArray);
          std::vector<geometry_msgs::TransformStamped> transformArrary;
          num_bodies = body_frame.get_num_bodies();
          for (size_t i = 0; i < num_bodies; ++i)
          {
            //tf2_ros::TransformListener tfListener(tfBuffer);
            k4abt_body_t body = body_frame.get_body(i);
            for (int j = 0; j < (int) K4ABT_JOINT_COUNT; ++j)
            {
              MarkerPtr markerPtr(new Marker);
              geometry_msgs::TransformStamped transform_msg; 
              getBodyMarker(body, markerPtr, transform_msg, i, j, capture_time);
              markerArrayPtr->markers.push_back(*markerPtr);
              transformArrary.push_back(std::move(transform_msg));
            }
          }
          body_marker_publisher_.publish(markerArrayPtr);
          br.sendTransform(transformArrary); 
        }

        if (body_index_map_publisher_.getNumSubscribers() > 0)
        {
          // Body index map
          ImagePtr body_index_map_frame(new Image);
          auto result = getBodyIndexMap(body_frame, body_index_map_frame);

          if (result != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR_STREAM("Failed to get body index map");
            ros::shutdown();
            return;
          }
          else if (result == K4A_RESULT_SUCCEEDED)
          {
            // Re-sychronize the timestamps with the capture timestamp
            body_index_map_frame->header.stamp = capture_time;
            body_index_map_frame->header.frame_id =
                calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;

            body_index_map_publisher_.publish(body_index_map_frame);
          }
        }
      }
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds{ 20 });
    }
  }
}
#endif

k4a_imu_sample_t K4AROSDevice::computeMeanIMUSample(const std::vector<k4a_imu_sample_t>& samples)
{
  // Compute mean sample
  // Using double-precision version of imu sample struct to avoid overflow
  k4a_imu_accumulator_t mean;
  for (auto imu_sample : samples)
  {
    mean += imu_sample;
  }
  float num_samples = samples.size();
  mean /= num_samples;

  // Convert to floating point
  k4a_imu_sample_t mean_float;
  mean.to_float(mean_float);
  // Use most timestamp of most recent sample
  mean_float.acc_timestamp_usec = samples.back().acc_timestamp_usec;
  mean_float.gyro_timestamp_usec = samples.back().gyro_timestamp_usec;

  return mean_float;
}

void K4AROSDevice::imuPublisherThread()
{
  ros::Rate loop_rate(300);

  k4a_result_t result;
  k4a_imu_sample_t sample;

  // For IMU throttling
  unsigned int count = 0;
  unsigned int target_count = params_.imu_rate_target ? IMU_MAX_RATE / params_.imu_rate_target : IMU_MAX_RATE;
  std::vector<k4a_imu_sample_t> accumulated_samples;
  accumulated_samples.reserve(target_count);
  bool throttling = target_count > 1;

  while (running_ && ros::ok() && !ros::isShuttingDown())
  {
    if (k4a_device_)
    {
      // IMU messages are delivered in batches at 300 Hz. Drain the queue of IMU messages by
      // constantly reading until we get a timeout
      bool read = false;
      do
      {
        read = k4a_device_.get_imu_sample(&sample, std::chrono::milliseconds(0));

        if (read)
        {
          if (throttling)
          {
            accumulated_samples.push_back(sample);
            count++;
          }

          if (count % target_count == 0)
          {
            ImuPtr imu_msg(new Imu);

            if (throttling)
            {
              k4a_imu_sample_t mean_sample_float = computeMeanIMUSample(accumulated_samples);
              result = getImuFrame(mean_sample_float, imu_msg);
              accumulated_samples.clear();
              count = 0;
            }
            else
            {
              result = getImuFrame(sample, imu_msg);
            }

            ROS_ASSERT_MSG(result == K4A_RESULT_SUCCEEDED, "Failed to get IMU frame");

            if (std::abs(imu_msg->angular_velocity.x) > DBL_EPSILON ||
                std::abs(imu_msg->angular_velocity.y) > DBL_EPSILON ||
                std::abs(imu_msg->angular_velocity.z) > DBL_EPSILON){
              imu_orientation_publisher_.publish(imu_msg);
            }
          }
        }

      } while (read);
    }
    else if (k4a_playback_handle_)
    {
      // publish imu messages as long as the imu timestamp is less than the last capture timestamp to catch up to the
      // cameras compare signed with unsigned shouldn't cause a problem because timestamps should always be positive
      while (last_imu_time_usec_ <= last_capture_time_usec_ && !imu_stream_end_of_file_)
      {
        std::lock_guard<std::mutex> guard(k4a_playback_handle_mutex_);
        if (!k4a_playback_handle_.get_next_imu_sample(&sample))
        {
          imu_stream_end_of_file_ = true;
        }
        else
        {
          if (throttling)
          {
            accumulated_samples.push_back(sample);
            count++;
          }

          if (count % target_count == 0)
          {
            ImuPtr imu_msg(new Imu);

            if (throttling)
            {
              k4a_imu_sample_t mean_sample_float = computeMeanIMUSample(accumulated_samples);
              result = getImuFrame(mean_sample_float, imu_msg);
              accumulated_samples.clear();
              count = 0;
            }
            else
            {
              result = getImuFrame(sample, imu_msg);
            }

            ROS_ASSERT_MSG(result == K4A_RESULT_SUCCEEDED, "Failed to get IMU frame");

            if (std::abs(imu_msg->angular_velocity.x) > DBL_EPSILON ||
                std::abs(imu_msg->angular_velocity.y) > DBL_EPSILON ||
                std::abs(imu_msg->angular_velocity.z) > DBL_EPSILON){
              imu_orientation_publisher_.publish(imu_msg);
            }

            last_imu_time_usec_ = sample.acc_timestamp_usec;
          }
        }
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

std::chrono::microseconds K4AROSDevice::getCaptureTimestamp(const k4a::capture& capture)
{
  // Captures don't actually have timestamps, images do, so we have to look at all the images
  // associated with the capture.  We just return the first one we get back.
  //
  // We check the IR capture instead of the depth capture because if the depth camera is started
  // in passive IR mode, it only has an IR image (i.e. no depth image), but there is no mode
  // where a capture will have a depth image but not an IR image.
  //
  const auto irImage = capture.get_ir_image();
  if (irImage != nullptr)
  {
    return irImage.get_device_timestamp();
  }

  const auto colorImage = capture.get_color_image();
  if (colorImage != nullptr)
  {
    return colorImage.get_device_timestamp();
  }

  return std::chrono::microseconds::zero();
}

// Converts a k4a *device* timestamp to a ros::Time object
ros::Time K4AROSDevice::timestampToROS(const std::chrono::microseconds& k4a_timestamp_us)
{
  // This will give INCORRECT timestamps until the first image.
  if (device_to_realtime_offset_.count() == 0)
  {
    initializeTimestampOffset(k4a_timestamp_us);
  }

  std::chrono::nanoseconds timestamp_in_realtime = k4a_timestamp_us + device_to_realtime_offset_;
  ros::Time ros_time;
  ros_time.fromNSec(timestamp_in_realtime.count());
  return ros_time;
}

// Converts a k4a_imu_sample_t timestamp to a ros::Time object
ros::Time K4AROSDevice::timestampToROS(const uint64_t& k4a_timestamp_us)
{
  return timestampToROS(std::chrono::microseconds(k4a_timestamp_us));
}

void K4AROSDevice::initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us)
{
  // We have no better guess than "now".
  std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();

  device_to_realtime_offset_ = realtime_clock - k4a_device_timestamp_us;

  ROS_WARN_STREAM("Initializing the device to realtime offset based on wall clock: "
                  << device_to_realtime_offset_.count() << " ns");
}

void K4AROSDevice::updateTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us,
                                         const std::chrono::nanoseconds& k4a_system_timestamp_ns)
{
  // System timestamp is on monotonic system clock.
  // Device time is on AKDK hardware clock.
  // We want to continuously estimate diff between realtime and AKDK hardware clock as low-pass offset.
  // This consists of two parts: device to monotonic, and monotonic to realtime.

  // First figure out realtime to monotonic offset. This will change to keep updating it.
  std::chrono::nanoseconds realtime_clock = std::chrono::system_clock::now().time_since_epoch();
  std::chrono::nanoseconds monotonic_clock = std::chrono::steady_clock::now().time_since_epoch();

  std::chrono::nanoseconds monotonic_to_realtime = realtime_clock - monotonic_clock;

  // Next figure out the other part (combined).
  std::chrono::nanoseconds device_to_realtime =
      k4a_system_timestamp_ns - k4a_device_timestamp_us + monotonic_to_realtime;
  // If we're over a second off, just snap into place.
  if (device_to_realtime_offset_.count() == 0 ||
      std::abs((device_to_realtime_offset_ - device_to_realtime).count()) > 1e7)
  {
    ROS_WARN_STREAM("Initializing or re-initializing the device to realtime offset: " << device_to_realtime.count()
                                                                                      << " ns");
    device_to_realtime_offset_ = device_to_realtime;
  }
  else
  {
    // Low-pass filter!
    constexpr double alpha = 0.10;
    device_to_realtime_offset_ = device_to_realtime_offset_ +
                                 std::chrono::nanoseconds(static_cast<int64_t>(
                                     std::floor(alpha * (device_to_realtime - device_to_realtime_offset_).count())));
  }
}
