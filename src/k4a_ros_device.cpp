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
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <angles/angles.h>
#include <k4a/k4a.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_types.h"
#include "azure_kinect_ros_driver/k4a.hpp"

using namespace ros;
using namespace sensor_msgs;
using namespace image_transport;
using namespace std;

K4AROSDevice::K4AROSDevice(const NodeHandle &n, const NodeHandle &p) : k4a_device_(nullptr),
                                                                       node_(n),
                                                                       private_node_(p),
                                                                       image_transport_(n)
{
    // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) private_node_.param(#param_variable, params_.param_variable, param_default_val);
    ROS_PARAM_LIST
#undef LIST_ENTRY

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
        catch(exception)
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

    ROS_INFO_STREAM("K4A Serial Number: " << k4a_device_.get_serialnum());

    k4a_hardware_version_t version_info = k4a_device_.get_version();

    ROS_INFO(
        "RGB Version: %d.%d.%d",
        version_info.rgb.major,
        version_info.rgb.minor,
        version_info.rgb.iteration);

    ROS_INFO(
        "Depth Version: %d.%d.%d",
        version_info.depth.major,
        version_info.depth.minor,
        version_info.depth.iteration);

    ROS_INFO(
        "Audio Version: %d.%d.%d",
        version_info.audio.major,
        version_info.audio.minor,
        version_info.audio.iteration);

    ROS_INFO(
        "Depth Sensor Version: %d.%d.%d",
        version_info.depth_sensor.major,
        version_info.depth_sensor.minor,
        version_info.depth_sensor.iteration);
    
    // Register our topics
    rgb_raw_publisher_ = image_transport_.advertise("rgb/image_raw", 1);
    rgb_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("rgb/camera_info", 1);

    depth_raw_publisher_ = image_transport_.advertise("depth/image_raw", 1);
    depth_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("depth/camera_info", 1);

    depth_rect_publisher_ = image_transport_.advertise("depth_to_rgb/image_raw", 1);
    depth_rect_camerainfo_publisher_ = node_.advertise<CameraInfo>("depth_to_rgb/camera_info", 1);

    rgb_rect_publisher_ = image_transport_.advertise("rgb_to_depth/image_raw", 1);
    rgb_rect_camerainfo_publisher_ = node_.advertise<CameraInfo>("rgb_to_depth/camera_info", 1);

    ir_raw_publisher_ = image_transport_.advertise("ir/image_raw", 1);
    ir_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("ir/camera_info", 1);

    imu_orientation_publisher_ = node_.advertise<Imu>("imu", 200);

    pointcloud_publisher_ = node_.advertise<PointCloud2>("points2", 1);
}

K4AROSDevice::~K4AROSDevice()
{
    // Start tearing down the publisher threads
    running_ = false;

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
}


k4a_result_t K4AROSDevice::startCameras()
{
    k4a_device_configuration_t k4a_configuration = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    k4a_result_t result = params_.GetDeviceConfig(&k4a_configuration);

    if (result != K4A_RESULT_SUCCEEDED)
    {
        ROS_ERROR("Failed to generate a device configuration. Not starting camera!");
        return result;
    }

    // Now that we have a proposed camera configuration, we can 
    // initialize the class which will take care of device calibration information
    calibration_data_.initialize(k4a_device_, k4a_configuration.depth_mode, k4a_configuration.color_resolution, params_);

    ROS_INFO_STREAM("STARTING CAMERAS");
    k4a_device_.start_cameras(&k4a_configuration);

    // Cannot assume the device timestamp begins increasing upon starting the cameras. 
    // If we set the time base here, depending on the machine performance, the new timestamp 
    // would lag the value of ros::Time::now() by at least 0.5 secs which is much larger than 
    // the real transmission delay as can be observed using the rqt_plot tool.
    // start_time_ = ros::Time::now();

    // Prevent the worker thread from exiting immediately
    running_ = true;

    // Start the thread that will poll the cameras and publish frames
    frame_publisher_thread_ = thread(&K4AROSDevice::framePublisherThread, this);

    return K4A_RESULT_SUCCEEDED;
}


k4a_result_t K4AROSDevice::startImu()
{
    ROS_INFO_STREAM("STARTING IMU");
    k4a_device_.start_imu();

    // Start the IMU publisher thread
    imu_publisher_thread_ = thread(&K4AROSDevice::imuPublisherThread, this);

    return K4A_RESULT_SUCCEEDED;
}


void K4AROSDevice::stopCameras()
{
    // Stop the K4A SDK
    ROS_INFO("Stopping K4A device");
    k4a_device_.stop_cameras();
    ROS_INFO("K4A device stopped");
}


void K4AROSDevice::stopImu()
{
    k4a_device_.stop_imu();
}


k4a_result_t K4AROSDevice::getDepthFrame(const k4a::capture &capture, sensor_msgs::ImagePtr depth_image, bool rectified = false)
{
    k4a::image k4a_depth_frame = capture.get_depth_image();

    if (!k4a_depth_frame)
    {
        ROS_ERROR("Cannot render depth frame: no frame");
        return K4A_RESULT_FAILED;
    }
     
    if(rectified)
    {
        calibration_data_.k4a_transformation_.depth_image_to_color_camera(
            k4a_depth_frame,
            &calibration_data_.transformed_depth_image_);

        return renderDepthToROS(depth_image, calibration_data_.transformed_depth_image_);
    }

    return renderDepthToROS(depth_image, k4a_depth_frame);
}


k4a_result_t K4AROSDevice::renderDepthToROS(sensor_msgs::ImagePtr depth_image, k4a::image& k4a_depth_frame)
{
    // Compute the expected size of the frame and compare to the actual frame size in bytes
    size_t depth_source_size = static_cast<size_t>(k4a_depth_frame.get_width_pixels() * k4a_depth_frame.get_height_pixels()) * sizeof(DepthPixel);

    // Access the depth image as an array of uint16 pixels
    DepthPixel* depth_frame_buffer = reinterpret_cast<DepthPixel *>(k4a_depth_frame.get_buffer());
    size_t depth_pixel_count = depth_source_size / sizeof(DepthPixel);

    // Build the ROS message
    depth_image->height = k4a_depth_frame.get_height_pixels();
    depth_image->width = k4a_depth_frame.get_width_pixels();
    depth_image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depth_image->is_bigendian = false;
    depth_image->step = k4a_depth_frame.get_width_pixels() * sizeof(float);

    // Enlarge the data buffer in the ROS message to hold the frame
    depth_image->data.resize(depth_image->height * depth_image->step);

    float* depth_image_data = reinterpret_cast<float*>(&depth_image->data[0]);

    // Copy the depth pixels into the ROS message, transforming them to floats at the same time
    // TODO: can this be done faster?
    for (size_t i = 0; i < depth_pixel_count; i++)
    {
        depth_image_data[i] = static_cast<float>(depth_frame_buffer[i]);

        if (depth_image_data[i] <= 0.f)
        {
            depth_image_data[i] = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            depth_image_data[i] /= 1000.0f;
        }
    }

    return K4A_RESULT_SUCCEEDED;
}


k4a_result_t K4AROSDevice::getIrFrame(const k4a::capture &capture, sensor_msgs::ImagePtr ir_image)
{
    k4a::image k4a_ir_frame = capture.get_ir_image();

    if (!k4a_ir_frame)
    {
        ROS_ERROR("Cannot render IR frame: no frame");
        return K4A_RESULT_FAILED;
    }

    return renderIrToROS(ir_image, k4a_ir_frame);
}


k4a_result_t K4AROSDevice::renderIrToROS(sensor_msgs::ImagePtr ir_image, k4a::image& k4a_ir_frame)
{
    // Compute the expected size of the frame and compare to the actual frame size in bytes
    size_t ir_source_size = static_cast<size_t>(k4a_ir_frame.get_width_pixels() * k4a_ir_frame.get_height_pixels()) * sizeof(IrPixel);

    // Access the ir image as an array of uint16 pixels
    IrPixel* ir_frame_buffer = reinterpret_cast<IrPixel *>(k4a_ir_frame.get_buffer());
    size_t ir_pixel_count = ir_source_size / sizeof(IrPixel);

    // Build the ROS message
    ir_image->height = k4a_ir_frame.get_height_pixels();
    ir_image->width = k4a_ir_frame.get_width_pixels();
    ir_image->encoding = sensor_msgs::image_encodings::MONO16;
    ir_image->is_bigendian = false;
    ir_image->step = k4a_ir_frame.get_width_pixels() * sizeof(IrPixel);

    // Enlarge the data buffer in the ROS message to hold the frame
    ir_image->data.resize(ir_image->height * ir_image->step);
    
    IrPixel* ir_image_data = reinterpret_cast<IrPixel*>(&ir_image->data[0]);

    memcpy(ir_image_data, k4a_ir_frame.get_buffer(), ir_image->height * ir_image->step);

    return K4A_RESULT_SUCCEEDED;
}


k4a_result_t K4AROSDevice::getRbgFrame(const k4a::capture &capture, sensor_msgs::ImagePtr rgb_image, bool rectified = false)
{
    k4a::image k4a_bgra_frame = capture.get_color_image();

    if (!k4a_bgra_frame)
    {
        ROS_ERROR("Cannot render BGRA frame: no frame");
        return K4A_RESULT_FAILED;
    }

    size_t color_image_size = static_cast<size_t>(k4a_bgra_frame.get_width_pixels() * k4a_bgra_frame.get_height_pixels()) * sizeof(BgraPixel);

    if (k4a_bgra_frame.get_size() != color_image_size)
    {
        ROS_WARN("Invalid k4a_bgra_frame returned from K4A");
        return K4A_RESULT_FAILED;
    }

    if (rectified)
    {
        k4a::image k4a_depth_frame = capture.get_depth_image();

        calibration_data_.k4a_transformation_.color_image_to_depth_camera(
            k4a_depth_frame,
            k4a_bgra_frame,
            &calibration_data_.transformed_rgb_image_);

        return renderBGRA32ToROS(rgb_image, calibration_data_.transformed_rgb_image_);
    }   

    return renderBGRA32ToROS(rgb_image, k4a_bgra_frame);
}


// Helper function that renders any BGRA K4A frame to a ROS ImagePtr. Useful for rendering intermediary frames
// during debugging of image processing functions
k4a_result_t K4AROSDevice::renderBGRA32ToROS(sensor_msgs::ImagePtr rgb_image, k4a::image& k4a_bgra_frame)
{
    size_t color_image_size = static_cast<size_t>(k4a_bgra_frame.get_width_pixels() * k4a_bgra_frame.get_height_pixels()) * sizeof(BgraPixel);

    // Build the ROS message
    rgb_image->height = k4a_bgra_frame.get_height_pixels();
    rgb_image->width = k4a_bgra_frame.get_width_pixels();
    rgb_image->encoding = sensor_msgs::image_encodings::BGRA8;
    rgb_image->is_bigendian = false;
    rgb_image->step = k4a_bgra_frame.get_width_pixels() * sizeof(BgraPixel);

    // Enlarge the data buffer in the ROS message to hold the frame
    rgb_image->data.resize(rgb_image->height * rgb_image->step);

    ROS_ASSERT_MSG(color_image_size == rgb_image->height * rgb_image->step, "Pixel buffer and ROS message buffer sizes are different");

    uint8_t *rgb_image_data = reinterpret_cast<uint8_t *>(&rgb_image->data[0]);
    uint8_t *bgra_frame_buffer = k4a_bgra_frame.get_buffer();
    
    // Copy memory from the Azure Kinect buffer into the ROS buffer
    memcpy(rgb_image_data, bgra_frame_buffer, rgb_image->height * rgb_image->step);

    return K4A_RESULT_SUCCEEDED;
}


k4a_result_t K4AROSDevice::getRgbPointCloud(const k4a::capture &capture, sensor_msgs::PointCloud2Ptr point_cloud)
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

    point_cloud->header.frame_id = calibration_data_.rgb_camera_frame_;
    point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
    printTimestampDebugMessage("RGB point cloud", point_cloud->header.stamp);
    point_cloud->height = k4a_bgra_frame.get_height_pixels();
    point_cloud->width = k4a_bgra_frame.get_width_pixels();
    point_cloud->is_dense = false;
    point_cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*point_cloud, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*point_cloud, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*point_cloud, "b");

    const size_t point_count = k4a_bgra_frame.get_size() / sizeof(BgraPixel);
    if (point_count != (point_cloud->height * point_cloud->width))
    {
        ROS_WARN("point_count does not match point cloud resolution!");
        return K4A_RESULT_FAILED;
    }

    const size_t pixel_count = k4a_bgra_frame.get_size() / sizeof(BgraPixel);
    if (pixel_count != (k4a_bgra_frame.get_height_pixels() * k4a_bgra_frame.get_width_pixels()))
    {
        ROS_WARN("pixel_count does not match RGB image resolution!");
        return K4A_RESULT_FAILED;
    }

    // transform depth image into color camera geometry
    calibration_data_.k4a_transformation_.depth_image_to_color_camera(
        k4a_depth_frame,
        &calibration_data_.transformed_depth_image_);

    // Tranform depth image to point cloud (note that this is now from the perspective of the color camera)
    calibration_data_.k4a_transformation_.depth_image_to_point_cloud(
        calibration_data_.transformed_depth_image_,
        K4A_CALIBRATION_TYPE_COLOR,
        &calibration_data_.point_cloud_image_);

    int16_t *point_cloud_buffer = reinterpret_cast<int16_t *>(calibration_data_.point_cloud_image_.get_buffer());
    uint8_t *rgb_image_buffer = k4a_bgra_frame.get_buffer();

    for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
    {
        k4a_float3_t depth_point_3d;
        depth_point_3d.xyz.z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

        // Get RGB values from the transformed image buffer
        BgraPixel color_pixel = {rgb_image_buffer[4 * i + 0],
                                 rgb_image_buffer[4 * i + 1],
                                 rgb_image_buffer[4 * i + 2],
                                 rgb_image_buffer[4 * i + 3]};

        if ((depth_point_3d.xyz.z <= 0.f) ||
            ((color_pixel.Red == 0) && (color_pixel.Green == 0) && (color_pixel.Blue == 0)))
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
            *iter_r = *iter_g = *iter_b = 0;
        }
        else
        {
            depth_point_3d.xyz.x = float(point_cloud_buffer[3 * i + 0]);
            depth_point_3d.xyz.y = float(-1 * point_cloud_buffer[3 * i + 1]);

            // Points from the camera are measured in milimeters. ROS expects the floats to be in meters.
            // Divide all points by 1000 to convert from mm to m
            for (float &f : depth_point_3d.v)
            {
                f /= 1000.0f;
            }

            // K4A Depth co-ordinates:
            //  x+ = "right"
            //  y+ = "up"
            //  z+ = "forward"
            //
            // ROS Standard Camera co-ordinates:
            //  x+ = "right"
            //  y+ = "down"
            //  z+ = "forward"
            //
            // Remap K4A to ROS co-ordinate system:
            // ROS_X+ = K4A_Z+
            // ROS_Y+ = K4A_X-
            // ROS_Z+ = K4A_Y+

            *iter_x = 1.0 * depth_point_3d.xyz.x;
            *iter_y = -1.0 * depth_point_3d.xyz.y;
            *iter_z = 1.0 * depth_point_3d.xyz.z;

            *iter_r = color_pixel.Red;
            *iter_g = color_pixel.Green;
            *iter_b = color_pixel.Blue;
        }
    }

    return K4A_RESULT_SUCCEEDED;
}


k4a_result_t K4AROSDevice::getPointCloud(const k4a::capture &capture, sensor_msgs::PointCloud2Ptr point_cloud)
{
    k4a::image k4a_depth_frame = capture.get_depth_image();

    if (!k4a_depth_frame)
    {
        ROS_ERROR("Cannot render point cloud: no depth frame");
        return K4A_RESULT_FAILED;
    }

    point_cloud->header.frame_id = calibration_data_.depth_camera_frame_;
    point_cloud->header.stamp = timestampToROS(k4a_depth_frame.get_device_timestamp());
    printTimestampDebugMessage("Point cloud", point_cloud->header.stamp);

    point_cloud->height = k4a_depth_frame.get_height_pixels();
    point_cloud->width = k4a_depth_frame.get_width_pixels();
    point_cloud->is_dense = false;
    point_cloud->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

    const size_t point_count = k4a_depth_frame.get_size() / sizeof(DepthPixel);

    if (point_count != (point_cloud->height * point_cloud->width))
    {
        ROS_WARN("point_count does not match point cloud resolution!");
    }

    // Tranform depth image to point cloud
    calibration_data_.k4a_transformation_.depth_image_to_point_cloud(
        k4a_depth_frame,
        K4A_CALIBRATION_TYPE_DEPTH,
        &calibration_data_.point_cloud_image_);

    int16_t *point_cloud_buffer = reinterpret_cast<int16_t *>(calibration_data_.point_cloud_image_.get_buffer());

    for (size_t i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
    {
        k4a_float3_t depth_point_3d;
        depth_point_3d.xyz.z = static_cast<float>(point_cloud_buffer[3 * i + 2]);

        if (depth_point_3d.xyz.z <= 0.f)
        {
            *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
            depth_point_3d.xyz.x = float(point_cloud_buffer[3 * i]);
            depth_point_3d.xyz.y = float(-1 * point_cloud_buffer[3 * i + 1]);

            // Points from the camera are measured in milimeters. ROS expects the floats to be in meters.
            // Divide all points by 1000 to convert from mm to m
            for (float &f : depth_point_3d.v)
            {
                f /= 1000.0f;
            }

            // K4A Depth co-ordinates:
            //  x+ = "right"
            //  y+ = "up"
            //  z+ = "forward"
            //
            // ROS Standard Camera co-ordinates:
            //  x+ = "right"
            //  y+ = "down"
            //  z+ = "forward"
            //
            // Remap K4A to ROS co-ordinate system:
            // ROS_X+ = K4A_Z+
            // ROS_Y+ = K4A_X-
            // ROS_Z+ = K4A_Y+

            *iter_x =  1.0 * depth_point_3d.xyz.x;
            *iter_y = -1.0 * depth_point_3d.xyz.y;
            *iter_z =  1.0 * depth_point_3d.xyz.z;
        }
    }

    return K4A_RESULT_SUCCEEDED;
}


k4a_result_t K4AROSDevice::getImuFrame(const k4a_imu_sample_t& sample, sensor_msgs::ImuPtr imu_msg)
{
    imu_msg->header.frame_id = calibration_data_.imu_frame_;
    imu_msg->header.stamp = timestampToROS(sample.acc_timestamp_usec);
    printTimestampDebugMessage("IMU", imu_msg->header.stamp);

    // K4A IMU Co-ordinates
    //  x+ = "backwards"
    //  y+ = "left"
    //  z+ = "down"
    //
    // ROS Standard co-ordinates:
    //  x+ = "forward"
    //  y+ = "left"
    //  z+ = "up"
    //
    // Remap K4A IMU to ROS co-ordinate system:
    // ROS_X+ = K4A_X-
    // ROS_Y+ = K4A_Y+
    // ROS_Z+ = K4A_Z-

    // The K4A and ROS IMU frames don't agree. Fix that here
    imu_msg->angular_velocity.x = -1.0 * sample.gyro_sample.xyz.x;
    imu_msg->angular_velocity.y =  1.0 * sample.gyro_sample.xyz.y;
    imu_msg->angular_velocity.z = -1.0 * sample.gyro_sample.xyz.z;

    imu_msg->linear_acceleration.x = -1.0 * sample.acc_sample.xyz.x;
    imu_msg->linear_acceleration.y =  1.0 * sample.acc_sample.xyz.y;
    imu_msg->linear_acceleration.z = -1.0 * sample.acc_sample.xyz.z;

    // Disable the orientation component of the IMU message since it's invalid
    imu_msg->orientation_covariance[0] = -1.0;

    return K4A_RESULT_SUCCEEDED;
}


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

    calibration_data_.getDepthCameraInfo(depth_raw_camera_info);
    calibration_data_.getRgbCameraInfo(rgb_raw_camera_info);
    calibration_data_.getDepthCameraInfo(rgb_rect_camera_info);
    calibration_data_.getRgbCameraInfo(depth_rect_camera_info);
    calibration_data_.getDepthCameraInfo(ir_raw_camera_info);

    while (running_ && ros::ok() && !ros::isShuttingDown())
    {
        // TODO: consider appropriate capture timeout based on camera framerate
        bool success = k4a_device_.get_capture(&capture, std::chrono::milliseconds(K4A_WAIT_INFINITE));

        if (!success)
        {
            ROS_FATAL("Failed to poll cameras: node cannot continue.");
            ros::requestShutdown();
            return;
        }

        ImagePtr rgb_raw_frame(new Image);
        ImagePtr rgb_rect_frame(new Image);
        ImagePtr depth_raw_frame(new Image);
        ImagePtr depth_rect_frame(new Image);
        ImagePtr ir_raw_frame(new Image);
        PointCloud2Ptr point_cloud(new PointCloud2);

        if (params_.depth_enabled)
        {
            // Only do compute if we have subscribers
            if (ir_raw_publisher_.getNumSubscribers() > 0 || ir_raw_camerainfo_publisher_.getNumSubscribers() > 0)
            {
                // IR images are available in all depth modes
                result = getIrFrame(capture, ir_raw_frame);
                
                if (result != K4A_RESULT_SUCCEEDED)
                {
                    ROS_ERROR_STREAM("Failed to get raw IR frame");
                    ros::shutdown();
                    return;
                }

                capture_time = timestampToROS(capture.get_ir_image().get_device_timestamp());
                printTimestampDebugMessage("IR image", capture_time);

                // Re-sychronize the timestamps with the capture timestamp
                ir_raw_camera_info.header.stamp = capture_time;
                ir_raw_frame->header.stamp = capture_time;
                ir_raw_frame->header.frame_id = calibration_data_.depth_camera_frame_;

                ir_raw_publisher_.publish(ir_raw_frame);
                ir_raw_camerainfo_publisher_.publish(ir_raw_camera_info);
            }

            // Depth images are not available in PASSIVE_IR mode
            if (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR)
            {
                capture_time = timestampToROS(capture.get_depth_image().get_device_timestamp());
                printTimestampDebugMessage("Depth image", capture_time);

                if (depth_raw_publisher_.getNumSubscribers() > 0 || depth_raw_camerainfo_publisher_.getNumSubscribers() > 0)
                {
                    result = getDepthFrame(capture, depth_raw_frame);
            
                    if (result != K4A_RESULT_SUCCEEDED)
                    {
                        ROS_ERROR_STREAM("Failed to get raw depth frame");
                        ros::shutdown();
                        return;
                    }

                    // Re-sychronize the timestamps with the capture timestamp
                    depth_raw_camera_info.header.stamp = capture_time;
                    depth_raw_frame->header.stamp = capture_time;
                    depth_raw_frame->header.frame_id = calibration_data_.depth_camera_frame_;

                    depth_raw_publisher_.publish(depth_raw_frame);
                    depth_raw_camerainfo_publisher_.publish(depth_raw_camera_info);
                }
                
                // We can only rectify the depth into the color co-ordinates if the color camera is enabled!
                if (params_.color_enabled && (depth_rect_publisher_.getNumSubscribers() > 0 || depth_rect_camerainfo_publisher_.getNumSubscribers() > 0))
                {
                    result = getDepthFrame(capture, depth_rect_frame, true /* rectified */);
                
                    if (result != K4A_RESULT_SUCCEEDED)
                    {
                        ROS_ERROR_STREAM("Failed to get rectifed depth frame");
                        ros::shutdown();
                        return;
                    }

                    depth_rect_frame->header.stamp = capture_time;
                    depth_rect_frame->header.frame_id = calibration_data_.rgb_camera_frame_;
                    depth_rect_publisher_.publish(depth_rect_frame);

                    // Re-synchronize the header timestamps since we cache the camera calibration message
                    depth_rect_camera_info.header.stamp = capture_time;
                    depth_rect_camerainfo_publisher_.publish(depth_rect_camera_info);
                }
            }            
        }

        if (params_.color_enabled)
        {
            capture_time = timestampToROS(capture.get_color_image().get_device_timestamp());
            printTimestampDebugMessage("Color image", capture_time);
            
            if (rgb_raw_publisher_.getNumSubscribers() > 0 || rgb_raw_camerainfo_publisher_.getNumSubscribers() > 0)
            {
                result = getRbgFrame(capture, rgb_raw_frame);

                if (result != K4A_RESULT_SUCCEEDED)
                {
                    ROS_ERROR_STREAM("Failed to get RGB frame");
                    ros::shutdown();
                    return;
                }

                rgb_raw_frame->header.stamp =  capture_time;
                rgb_raw_frame->header.frame_id = calibration_data_.rgb_camera_frame_;
                rgb_raw_publisher_.publish(rgb_raw_frame);

                // Re-synchronize the header timestamps since we cache the camera calibration message
                rgb_raw_camera_info.header.stamp = capture_time;
                rgb_raw_camerainfo_publisher_.publish(rgb_raw_camera_info);
            }

            // We can only rectify the color into the depth co-ordinates if the depth camera is 
            // enabled and processing depth data
            if (params_.depth_enabled && 
                (calibration_data_.k4a_calibration_.depth_mode != K4A_DEPTH_MODE_PASSIVE_IR) && 
                (rgb_rect_publisher_.getNumSubscribers() > 0 || rgb_rect_camerainfo_publisher_.getNumSubscribers() > 0))
            {
                result = getRbgFrame(capture, rgb_rect_frame, true /* rectified */);
            
                if (result != K4A_RESULT_SUCCEEDED)
                {
                    ROS_ERROR_STREAM("Failed to get rectifed depth frame");
                    ros::shutdown();
                    return;
                }

                rgb_rect_frame->header.stamp = capture_time;
                rgb_rect_frame->header.frame_id = calibration_data_.depth_camera_frame_;
                rgb_rect_publisher_.publish(rgb_rect_frame);

                // Re-synchronize the header timestamps since we cache the camera calibration message
                rgb_rect_camera_info.header.stamp = capture_time;
                rgb_rect_camerainfo_publisher_.publish(rgb_rect_camera_info);
            }
        }

        if (pointcloud_publisher_.getNumSubscribers() > 0)
        {
            if (params_.rgb_point_cloud)
            {
                result = getRgbPointCloud(capture, point_cloud);

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
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}


void K4AROSDevice::imuPublisherThread()
{
    ros::Rate loop_rate(300);

    k4a_wait_result_t waitResult;
    k4a_result_t result;

    k4a_imu_sample_t sample;

    while (running_ && ros::ok() && !ros::isShuttingDown())
    {
        // TODO: consider appropriate capture timeout based on camera framerate
        bool success = k4a_device_.get_imu_sample(&sample, std::chrono::milliseconds(K4A_WAIT_INFINITE));

        if (!success)
        {
            ROS_FATAL("Failed to poll IMU: node cannot continue.");
            ros::requestShutdown();
            return;
        }

        ImuPtr imu_msg(new Imu);

        result = getImuFrame(sample, imu_msg);
        ROS_ASSERT_MSG(result == K4A_RESULT_SUCCEEDED, "Failed to get IMU frame");

        imu_orientation_publisher_.publish(imu_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

// Converts a k4a_image_t timestamp to a ros::Time object
ros::Time K4AROSDevice::timestampToROS(const std::chrono::microseconds & k4a_timestamp_us) 
{
    ros::Duration duration_since_device_startup(std::chrono::duration<double>(k4a_timestamp_us).count());

    // Set the time base if it is not set yet. Possible race condition should cause no harm.
    if (start_time_.isZero())
    {
        const ros::Duration transmission_delay(0.11);
        ROS_WARN_STREAM("Setting the time base using a k4a_image_t timestamp. This will result in a "
            "larger uncertainty than setting the time base using the timestamp of a k4a_imu_sample_t sample. "
            "Assuming the transmission delay to be " << transmission_delay.toSec() * 1000.0  << " ms."); 
        start_time_ = ros::Time::now() - duration_since_device_startup - transmission_delay;
    }
    return start_time_ + duration_since_device_startup;
}

// Converts a k4a_imu_sample_t timestamp to a ros::Time object
ros::Time K4AROSDevice::timestampToROS(const uint64_t & k4a_timestamp_us) 
{
      ros::Duration duration_since_device_startup(k4a_timestamp_us / 1e6);

      // Set the time base if it is not set yet.
      if (start_time_.isZero())
      {
        const ros::Duration transmission_delay(0.005);
        ROS_INFO_STREAM("Setting the time base using a k4a_imu_sample_t sample. "
          "Assuming the transmission delay to be " << transmission_delay.toSec() * 1000.0 << " ms."); 
        start_time_ = ros::Time::now() - duration_since_device_startup - transmission_delay;
      }
      return start_time_ + duration_since_device_startup;
    }

void printTimestampDebugMessage(const std::string name, const ros::Time & timestamp)
{
    ros::Duration lag = ros::Time::now() - timestamp;
    static std::map<const std::string, std::pair<ros::Duration, ros::Duration>> map_min_max;
    auto it = map_min_max.find(name);
    if (it == map_min_max.end())
    {
        map_min_max[name] = std::make_pair(lag, lag);
        it = map_min_max.find(name);
    }
    else
    {
        auto & min_lag = it->second.first;
        auto & max_lag = it->second.second;
        if (lag < min_lag)
            min_lag = lag;
        if (lag > max_lag)
            max_lag = lag;
    }
    
    ROS_DEBUG_STREAM(name << " timestamp lags ros::Time::now() by\n" 
        << std::setw(23) << lag.toSec() * 1000.0 << " ms. "
        << "The lag ranges from " << it->second.first.toSec() * 1000.0 << "ms"
        <<" to " << it->second.second.toSec() * 1000.0 << "ms.");
}
