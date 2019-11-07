// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_calibration_transform_data.h"

// System headers
//
#include <stdexcept>

// Library headers
//
#include <angles/angles.h>
#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_types.h"

void K4ACalibrationTransformData::initialize(const k4a::device& device, const k4a_depth_mode_t depth_mode,
                                             const k4a_color_resolution_t resolution, const K4AROSDeviceParams params)
{
  k4a_calibration_ = device.get_calibration(depth_mode, resolution);
  initialize(params);
}

void K4ACalibrationTransformData::initialize(const k4a::playback& k4a_playback_handle, const K4AROSDeviceParams params)
{
  k4a_calibration_ = k4a_playback_handle.get_calibration();
  initialize(params);
}

void K4ACalibrationTransformData::initialize(const K4AROSDeviceParams params)
{
  k4a_transformation_ = k4a::transformation(k4a_calibration_);
  tf_prefix_ = params.tf_prefix;

  print();

  bool depthEnabled = (getDepthWidth() * getDepthHeight() > 0);
  bool colorEnabled = (getColorWidth() * getColorHeight() > 0);

  // Create a buffer to store the point cloud
  if (params.point_cloud && (!params.rgb_point_cloud || params.point_cloud_in_depth_frame))
  {
    point_cloud_image_ = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, getDepthWidth(), getDepthHeight(),
                                            getDepthWidth() * 3 * (int) sizeof(DepthPixel));
  }
  else if (params.point_cloud && params.rgb_point_cloud)
  {
    point_cloud_image_ = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, getColorWidth(), getColorHeight(),
                                            getColorWidth() * 3 * (int) sizeof(DepthPixel));
  }

  if (depthEnabled && colorEnabled)
  {
    // Create a buffer to store RGB images that are transformed into the depth camera geometry
    transformed_rgb_image_ = k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32, getDepthWidth(), getDepthHeight(),
                                                getDepthWidth() * (int) sizeof(BgraPixel));

    // Create a buffer to store depth images that are transformed into the RGB camera geometry
    transformed_depth_image_ = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, getColorWidth(), getColorHeight(),
                                                  getColorWidth() * (int) sizeof(DepthPixel));
  }

  // Publish various transforms needed by ROS.
  // We publish all TFs all the time, even if the respective sensor data
  // output is off. This allows us to just use the SDK calibrations, and one
  // cosmetic TF output between the depth and the base.
  publishDepthToBaseTf();
  publishImuToDepthTf();
  publishRgbToDepthTf();
}

int K4ACalibrationTransformData::getDepthWidth() { return k4a_calibration_.depth_camera_calibration.resolution_width; }

int K4ACalibrationTransformData::getDepthHeight()
{
  return k4a_calibration_.depth_camera_calibration.resolution_height;
}

int K4ACalibrationTransformData::getColorWidth() { return k4a_calibration_.color_camera_calibration.resolution_width; }

int K4ACalibrationTransformData::getColorHeight()
{
  return k4a_calibration_.color_camera_calibration.resolution_height;
}

void K4ACalibrationTransformData::print()
{
  ROS_INFO("K4A Calibration Blob:");
  ROS_INFO("\t Depth:");
  printCameraCalibration(k4a_calibration_.depth_camera_calibration);

  ROS_INFO("\t Color:");
  printCameraCalibration(k4a_calibration_.color_camera_calibration);

  ROS_INFO("\t IMU (Depth to Color):");
  printExtrinsics(k4a_calibration_.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR]);

  ROS_INFO("\t IMU (Depth to IMU):");
  printExtrinsics(k4a_calibration_.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_ACCEL]);

  ROS_INFO("\t IMU (IMU to Depth):");
  printExtrinsics(k4a_calibration_.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_DEPTH]);

  ROS_INFO("\t IMU (Color to IMU):");
  printExtrinsics(k4a_calibration_.extrinsics[K4A_CALIBRATION_TYPE_COLOR][K4A_CALIBRATION_TYPE_ACCEL]);

  ROS_INFO("\t IMU (IMU to Color):");
  printExtrinsics(k4a_calibration_.extrinsics[K4A_CALIBRATION_TYPE_ACCEL][K4A_CALIBRATION_TYPE_COLOR]);
}

void K4ACalibrationTransformData::printCameraCalibration(k4a_calibration_camera_t& calibration)
{
  printExtrinsics(calibration.extrinsics);

  ROS_INFO("\t\t Resolution:");
  ROS_INFO_STREAM("\t\t\t Width: " << calibration.resolution_width);
  ROS_INFO_STREAM("\t\t\t Height: " << calibration.resolution_height);

  ROS_INFO("\t\t Intrinsics:");
  ROS_INFO_STREAM("\t\t\t Model Type: " << calibration.intrinsics.type);
  ROS_INFO_STREAM("\t\t\t Parameter Count: " << calibration.intrinsics.parameter_count);
  ROS_INFO_STREAM("\t\t\t cx: " << calibration.intrinsics.parameters.param.cx);
  ROS_INFO_STREAM("\t\t\t cy: " << calibration.intrinsics.parameters.param.cy);
  ROS_INFO_STREAM("\t\t\t fx: " << calibration.intrinsics.parameters.param.fx);
  ROS_INFO_STREAM("\t\t\t fy: " << calibration.intrinsics.parameters.param.fy);
  ROS_INFO_STREAM("\t\t\t k1: " << calibration.intrinsics.parameters.param.k1);
  ROS_INFO_STREAM("\t\t\t k2: " << calibration.intrinsics.parameters.param.k2);
  ROS_INFO_STREAM("\t\t\t k3: " << calibration.intrinsics.parameters.param.k3);
  ROS_INFO_STREAM("\t\t\t k4: " << calibration.intrinsics.parameters.param.k4);
  ROS_INFO_STREAM("\t\t\t k5: " << calibration.intrinsics.parameters.param.k5);
  ROS_INFO_STREAM("\t\t\t k6: " << calibration.intrinsics.parameters.param.k6);
  ROS_INFO_STREAM("\t\t\t codx: " << calibration.intrinsics.parameters.param.codx);
  ROS_INFO_STREAM("\t\t\t cody: " << calibration.intrinsics.parameters.param.cody);
  ROS_INFO_STREAM("\t\t\t p2: " << calibration.intrinsics.parameters.param.p2);
  ROS_INFO_STREAM("\t\t\t p1: " << calibration.intrinsics.parameters.param.p1);
  ROS_INFO_STREAM("\t\t\t metric_radius: " << calibration.intrinsics.parameters.param.metric_radius);
}

void K4ACalibrationTransformData::printExtrinsics(k4a_calibration_extrinsics_t& extrinsics)
{
  ROS_INFO("\t\t Extrinsics:");
  ROS_INFO_STREAM("\t\t\t Translation: " << extrinsics.translation[0] << ", " << extrinsics.translation[1] << ", "
                                         << extrinsics.translation[2]);
  ROS_INFO_STREAM("\t\t\t Rotation[0]: " << extrinsics.rotation[0] << ", " << extrinsics.rotation[1] << ", "
                                         << extrinsics.rotation[2]);
  ROS_INFO_STREAM("\t\t\t Rotation[1]: " << extrinsics.rotation[3] << ", " << extrinsics.rotation[4] << ", "
                                         << extrinsics.rotation[5]);
  ROS_INFO_STREAM("\t\t\t Rotation[2]: " << extrinsics.rotation[6] << ", " << extrinsics.rotation[7] << ", "
                                         << extrinsics.rotation[8]);
}

void K4ACalibrationTransformData::publishRgbToDepthTf()
{
  k4a_calibration_extrinsics_t* rgb_extrinsics =
      &k4a_calibration_.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
  tf2::Vector3 depth_to_rgb_translation(rgb_extrinsics->translation[0] / 1000.0f,
                                        rgb_extrinsics->translation[1] / 1000.0f,
                                        rgb_extrinsics->translation[2] / 1000.0f);
  tf2::Matrix3x3 depth_to_rgb_rotation(
      rgb_extrinsics->rotation[0], rgb_extrinsics->rotation[1], rgb_extrinsics->rotation[2],
      rgb_extrinsics->rotation[3], rgb_extrinsics->rotation[4], rgb_extrinsics->rotation[5],
      rgb_extrinsics->rotation[6], rgb_extrinsics->rotation[7], rgb_extrinsics->rotation[8]);
  tf2::Transform depth_to_rgb_transform(depth_to_rgb_rotation, depth_to_rgb_translation);

  geometry_msgs::TransformStamped static_transform;
  static_transform.transform = tf2::toMsg(depth_to_rgb_transform.inverse());

  static_transform.header.stamp = ros::Time::now();
  static_transform.header.frame_id = tf_prefix_ + depth_camera_frame_;
  static_transform.child_frame_id = tf_prefix_ + rgb_camera_frame_;

  static_broadcaster_.sendTransform(static_transform);
}

void K4ACalibrationTransformData::publishImuToDepthTf()
{
  k4a_calibration_extrinsics_t* imu_extrinsics =
      &k4a_calibration_.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_ACCEL];
  tf2::Vector3 depth_to_imu_translation(imu_extrinsics->translation[0] / 1000.0f,
                                        imu_extrinsics->translation[1] / 1000.0f,
                                        imu_extrinsics->translation[2] / 1000.0f);
  tf2::Matrix3x3 depth_to_imu_rotation(
      imu_extrinsics->rotation[0], imu_extrinsics->rotation[1], imu_extrinsics->rotation[2],
      imu_extrinsics->rotation[3], imu_extrinsics->rotation[4], imu_extrinsics->rotation[5],
      imu_extrinsics->rotation[6], imu_extrinsics->rotation[7], imu_extrinsics->rotation[8]);
  tf2::Transform depth_to_imu_transform(depth_to_imu_rotation, depth_to_imu_translation);

  geometry_msgs::TransformStamped static_transform;
  static_transform.transform = tf2::toMsg(depth_to_imu_transform.inverse());

  static_transform.header.stamp = ros::Time::now();
  static_transform.header.frame_id = tf_prefix_ + depth_camera_frame_;
  static_transform.child_frame_id = tf_prefix_ + imu_frame_;

  static_broadcaster_.sendTransform(static_transform);
}

void K4ACalibrationTransformData::publishDepthToBaseTf()
{
  // This is a purely cosmetic transform to make the base model of the URDF look good.
  geometry_msgs::TransformStamped static_transform;

  static_transform.header.stamp = ros::Time::now();
  static_transform.header.frame_id = tf_prefix_ + camera_base_frame_;
  static_transform.child_frame_id = tf_prefix_ + depth_camera_frame_;

  tf2::Vector3 depth_translation = getDepthToBaseTranslationCorrection();
  static_transform.transform.translation.x = depth_translation.x();
  static_transform.transform.translation.y = depth_translation.y();
  static_transform.transform.translation.z = depth_translation.z();

  tf2::Quaternion depth_rotation = getDepthToBaseRotationCorrection();
  static_transform.transform.rotation.x = depth_rotation.x();
  static_transform.transform.rotation.y = depth_rotation.y();
  static_transform.transform.rotation.z = depth_rotation.z();
  static_transform.transform.rotation.w = depth_rotation.w();

  static_broadcaster_.sendTransform(static_transform);
}

// The [0,0,0] center of the URDF, the TF frame known as "camera_base", is offset slightly from the
// [0,0,0] origin of the depth camera frame, known as "depth_camera_link" or "depth_camera_frame"
//
// Publish a TF link so the URDF model and the depth camera line up correctly
#define DEPTH_CAMERA_OFFSET_MM_X 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Y 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Z 1.8f  // The depth camera is shifted 1.8mm up in the depth window

tf2::Vector3 K4ACalibrationTransformData::getDepthToBaseTranslationCorrection()
{
  // These are purely cosmetic tranformations for the URDF drawing!!
  return tf2::Vector3(DEPTH_CAMERA_OFFSET_MM_X / 1000.0f, DEPTH_CAMERA_OFFSET_MM_Y / 1000.0f,
                      DEPTH_CAMERA_OFFSET_MM_Z / 1000.0f);
}

tf2::Quaternion K4ACalibrationTransformData::getDepthToBaseRotationCorrection()
{
  // These are purely cosmetic tranformations for the URDF drawing!!
  tf2::Quaternion ros_camera_rotation;  // ROS camera co-ordinate system requires rotating the entire camera relative to
                                        // camera_base
  tf2::Quaternion depth_rotation;       // K4A has one physical camera that is about 6 degrees downward facing.

  depth_rotation.setEuler(0, angles::from_degrees(-6.0), 0);
  ros_camera_rotation.setEuler(M_PI / -2.0f, M_PI, (M_PI / 2.0f));

  return ros_camera_rotation * depth_rotation;
}

void K4ACalibrationTransformData::getDepthCameraInfo(sensor_msgs::CameraInfo& camera_info)
{
  camera_info.header.frame_id = tf_prefix_ + depth_camera_frame_;
  camera_info.width = getDepthWidth();
  camera_info.height = getDepthHeight();
  camera_info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  k4a_calibration_intrinsic_parameters_t* parameters = &k4a_calibration_.depth_camera_calibration.intrinsics.parameters;

  // The distortion parameters, size depending on the distortion model.
  // For "rational_polynomial", the 8 parameters are: (k1, k2, p1, p2, k3, k4, k5, k6).
  camera_info.D = {parameters->param.k1, parameters->param.k2, parameters->param.p1, parameters->param.p2,
                   parameters->param.k3, parameters->param.k4, parameters->param.k5, parameters->param.k6};

  // clang-format off
  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel
  // coordinates using the focal lengths (fx, fy) and principal point
  // (cx, cy).
  camera_info.K = {parameters->param.fx,  0.0f,                   parameters->param.cx,
                   0.0f,                  parameters->param.fy,   parameters->param.cy,
                   0.0f,                  0.0,                    1.0f};

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  // By convention, this matrix specifies the intrinsic (camera) matrix
  //  of the processed (rectified) image. That is, the left 3x3 portion
  //  is the normal camera intrinsic matrix for the rectified image.
  // It projects 3D points in the camera coordinate frame to 2D pixel
  //  coordinates using the focal lengths (fx', fy') and principal point
  //  (cx', cy') - these may differ from the values in K.
  // For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
  //  also have R = the identity and P[1:3,1:3] = K.
  camera_info.P = {parameters->param.fx,  0.0f,                   parameters->param.cx,   0.0f,
                   0.0f,                  parameters->param.fy,   parameters->param.cy,   0.0f,
                   0.0f,                  0.0,                    1.0f,                   0.0f};

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are
  // parallel.
  camera_info.R = {1.0f, 0.0f, 0.0f,
                   0.0f, 1.0f, 0.0f,
                   0.0f, 0.0f, 1.0f};
  // clang-format on
}

void K4ACalibrationTransformData::getRgbCameraInfo(sensor_msgs::CameraInfo& camera_info)
{
  camera_info.header.frame_id = tf_prefix_ + rgb_camera_frame_;
  camera_info.width = getColorWidth();
  camera_info.height = getColorHeight();
  camera_info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

  k4a_calibration_intrinsic_parameters_t* parameters = &k4a_calibration_.color_camera_calibration.intrinsics.parameters;

  // The distortion parameters, size depending on the distortion model.
  // For "rational_polynomial", the 8 parameters are: (k1, k2, p1, p2, k3, k4, k5, k6).
  camera_info.D = {parameters->param.k1, parameters->param.k2, parameters->param.p1, parameters->param.p2,
                   parameters->param.k3, parameters->param.k4, parameters->param.k5, parameters->param.k6};

  // clang-format off
  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel
  // coordinates using the focal lengths (fx, fy) and principal point
  // (cx, cy).
  camera_info.K = {parameters->param.fx,  0.0f,                   parameters->param.cx,
                   0.0f,                  parameters->param.fy,   parameters->param.cy,
                   0.0f,                  0.0,                    1.0f};

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  // By convention, this matrix specifies the intrinsic (camera) matrix
  //  of the processed (rectified) image. That is, the left 3x3 portion
  //  is the normal camera intrinsic matrix for the rectified image.
  // It projects 3D points in the camera coordinate frame to 2D pixel
  //  coordinates using the focal lengths (fx', fy') and principal point
  //  (cx', cy') - these may differ from the values in K.
  // For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
  //  also have R = the identity and P[1:3,1:3] = K.
  camera_info.P = {parameters->param.fx,  0.0f,                   parameters->param.cx,   0.0f,
                   0.0f,                  parameters->param.fy,   parameters->param.cy,   0.0f,
                   0.0f,                  0.0,                    1.0f,                   0.0f};

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are
  // parallel.
  camera_info.R = {1.0f, 0.0f, 0.0f,
                   0.0f, 1.0f, 0.0f,
                   0.0f, 0.0f, 1.0f};
  // clang-format on
}
