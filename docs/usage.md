# Azure Kinect ROS Driver Usage

The Azure Kinect ROS Driver node exposes Azure Kinect DK sensor streams to ROS.

## Starting the Driver

The ROS package contains two main launch files to start the driver:
- `driver.launch` starts the driver as a separate node which publishes the raw images and the factory calibration. **The published images are not rectified.**
- `kinect_rgbd.launch` starts the driver as a nodelet in a nodelet manager and starts the RGB-D processing from `rgbd_launch` which will load additional nodelets for image rectification and registration. This will publish raw (unrectified) as well as rectified images.

## Setup and Building

Please see the [building guide](building.md).

### Parameters

The Azure Kinect ROS Driver node accepts a number of [ROS Parameters](http://wiki.ros.org/Parameter%20Server) to configure the Azure Kinect DK sensor. Since the node uses the ROS parameter server, these parameters can be set in the usual ROS ways (on the command line, in a launch file, through the parameter server, etc..).

The node accepts the following parameters:

- `sensor_sn` (string) : No default value. The serial number of the Azure Kinect DK that the node should open. If this parameter is not specified, the node will auto-select the first Azure Kinect DK that it finds.
- `depth_enabled` (bool) : Default to '`true`'. Controls if the depth camera will be turned on.
- `depth_mode` (string) : Defaults to '`NFOV_UNBINNED`'. This string selects the depth camera operating mode. More details on the various depth camera modes can be found in the [Azure Kinect Sensor SDK documentation](https://docs.microsoft.com/en-us/azure/Kinect-dk/hardware-specification#depth-camera-supported-operating-modes). Valid options are '`NFOV_2X2BINNED`', '`NFOV_UNBINNED`', '`WFOV_2X2BINNED`', '`WFOV_UNBINNED`', '`PASSIVE_IR`'.
- `color_enabled` (bool) : Defaults to '`false`'. Controls if the color camera will be turned on.
- `color_resolution` (string) : Defaults to '`720P`'. This string selects the color camera resolution. The selected color camera resolution will affect the overlap between the depth and color camera field-of-view, which will impact the quality / resolution of the depth-to-color and color-to-depth point clouds. More details on the various color-camera resolutions and their aspect ratios can be found in the [Azure Kinect Sensor SDK documentation](https://docs.microsoft.com/en-us/azure/Kinect-dk/hardware-specification#color-camera-supported-operating-modes). Valid options are '`720P`', '`1080P`', '`1440P`', '`1536P`', '`2160P`', '`3072P`'.
- `fps` (int) : Defaults to `5`. This parameter controls the FPS of the color and depth cameras. The cameras cannot operate at different frame rates. Valid options are `5`, `15`, `30`. Note that some FPS values are not compatible with high color camera resolutions or depth camera resolutions. For more information, see the [Azure Kinect Sensor SDK documentation](https://docs.microsoft.com/en-us/azure/Kinect-dk/hardware-specification#depth-camera-supported-operating-modes).
- `point_cloud` (bool) : Defaults to `true`. If this parameter is set to `true`, the node will generate a sensor_msgs::PointCloud2 message from the depth camera data. This requires that the `depth_enabled` parameter be `true`.
- `rgb_point_cloud` (bool) : Defaults to `false`. If this parameter is set to `true`, the node will generate a sensor_msgs::PointCloud2 message from the depth camera data and colorize it using the color camera data. This requires that the `point_cloud` parameter be `true`, and the `color_enabled` parameter be `true`.
- `point_cloud_in_depth_frame` (bool) : Defaults to `true`. Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). Will either match the resolution of the depth camera (true) or the RGB camera (false).
- `recording_file` (string) : No default value. If this parameter contains a valid absolute path to a k4arecording file, the node will use the playback api with this file instead of opening a device.
- `recording_loop_enabled` (bool) : Defaults to `false`. If this parameter is set to `true`, the node will rewind the recording file to the beginning after reaching the last frame. Otherwise the node will stop working after reaching the end of the recording file.
- `body_tracking_enabled` (bool) : Defaults to `false`. If this parameter is set to `true`, the node will generate visualization_msgs::MarkerArray messages for the body tracking data. This requires that the `depth_enabled` parameter is set to `true` and an installed azure kinect body tracking sdk.
- `body_tracking_smoothing_factor` (float) : Defaults to `0.0`. Controls the temporal smoothing across frames. Set between `0` for no smoothing and `1` for full smoothing. Less smoothing will increase the responsiveness of the detected skeletons but will cause more positional and oriantational jitters.
- `rescale_ir_to_mono8` (bool) : Defaults to `false`. Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (`ir_mono8_scaling_factor`) is applied.
- `ir_mono8_scaling_factor` (float) : Defaults to `1.0`. Scaling factor to apply when converting IR to mono8 (see rescale_ir_to_mono8). If using illumination, use the value 0.5-1. If using passive IR, use 10.
- `imu_rate_target` (int) : Defaults to `0`. Controls the desired IMU message rate, which is rounded to the closest allowable value.  IMU samples from the device are integrated and a mean sample is published at this rate. A value of `0` is interpreted to mean a request for the maximum rate from the sensor (approx. 1.6 kHz).
- `wired_sync_mode` (int) : Defaults to `0`. Sets the external wired synchronization mode. The modes are: `0: OFF (STANDALONE)`, `1: MASTER`, `2: SUBORDINATE`.
- `subordinate_delay_off_master_usec` (int) : Defaults to `0`. Delay subordinate camera off master camera by specified amount in usec. Recommended minimum value is 160.
#### Parameter Restrictions

Some parameters are incompatible with each other. The ROS node attempts to detect incompatible parameters and provide a runtime error to roserr: however, not all potential incompatibilities have been accounted for. In these instances, the Azure Kinect Sensor SDK may throw an exception.

Some example incompatibilities are provided here:
- The `rgb_point_cloud` parameter requires that both the depth and color camera be enabled. This means that `depth_enabled` and `color_enabled` must both be `true`, `depth_mode`, `color_resolution`, and `fps` must be valid.
- Some combinations of depth mode / color resolution / FPS are incompatible. The Azure Kinect Sensor SDK will generate an exception and print a log message when this situation is detected. The ROS node will then exit.
- Enabling `rgb_point_cloud` will restrict the resolution of the emitted point cloud to the overlapping region between the depth and color cameras. This region is significantly smaller than the normal field-of-view of the depth camera.
- `recording_file` parameter accepts only absolute filepaths. Moreover color gets disabled if the color format is not BGRA32.

## Topics

The node emits a variety of topics into its namespace.

- `points2` (`sensor_msgs::PointCloud2`) : The point cloud generated by the Azure Kinect Sensor SDK from the depth camera data. If the `rgb_point_cloud` option is set, the points in the cloud will be colorized using information from the color camera.
- `rgb/image_raw` (`sensor_msgs::Image`) : The raw image from the color camera, in BGRA format. Note that this image is **not** undistorted. The image can be undistorted using the [image_proc package](http://wiki.ros.org/image_proc).
- `rgb/camera_info` (`sensor_msgs::CameraInfo`) : Calibration information for the color camera, converted from the Azure Kinect Sensor SDK format. The Azure Kinect DK uses the `rational_polynomial` distortion model.
- `depth/image_raw` (`sensor_msgs::Image`) : The raw image from the depth camera, in 32FC1 format. This differs from previous ROS Kinect drivers, which emitted data in the Kinect-native MONO16 format in units of millimeters. Note that this image is **not** undistorted. The image can be undistorted using the [image_proc package](http://wiki.ros.org/image_proc).
- `depth/camera_info` (`sensor_msgs::CameraInfo`) : Calibration information for the depth camera, converted from the Azure Kinect Sensor SDK format. The Azure Kinect DK uses the `rational_polynomial` distortion model.
- `depth_to_rgb/image_raw` (`sensor_msgs::Image`) :  The depth image, transformed into the color camera co-ordinate space by the Azure Kinect Sensor SDK. This image has been resized to match the color image resolution. Note that since the depth image is now transformed into the color camera co-ordinate space, some depth information may have been discarded if it was not visible to the depth camera.
- `depth_to_rgb/camera_info` (`sensor_msgs::CameraInfo`) : A copy of the color camera calibration which has been modified to match the depth co-ordinate frame. The depth camera image provided by `depth_to_rgb/image_raw` is distorted in the same way as the color camera image provided by `rgb/image_raw`. Correctly undistorting this image require the use of the color camera calibration, which is provided on this topic.
- `rgb_to_depth/image_raw` (`sensor_msgs::Image`) : The color image, transformed into the depth camera co-ordinate space by the Azure Kinect Sensor SDK. This image has been resized to match the depth image resolution, however depth pixels that are not visible to the color camera will return with a black color.
- `rgb_to_depth/camera_info` (`sensor_msgs::CameraInfo`) : A copy of the depth camera calibration which has been modified to match the color camera co-ordinate frame. The color camera image provided by `rgb_to_depth/image_raw` is distorted in the same way as the depth camera image provided by `depth/image_raw`. Correctly undistorting this image require the use of the depth camera calibration, which is provided on this topic.
- `ir/image_raw` (`sensor_msgs::Image`) : The raw infrared image from the depth camera sensor. In most depth modes, this image will be illuminated by the infrared illuminator built into the Azure Kinect DK. In `PASSIVE_IR` mode, this image will not be illuminated by the Azure Kinect DK. The encoding is uint16 by default, can be changed to be uint8 using the `rescale_ir_to_mono8` parameter.
- `ir/camera_info` (`sensor_msgs::CameraInfo`) : Calibration information for the infrared camera, converted from the Azure Kinect Sensor SDK format. Since the depth camera and infrared camera are physically the same camera, this `camera_info` is a copy of the camera info published for the depth camera.
- `imu` (`sensor_msgs::Imu`) : The intrinsics-corrected IMU sensor stream, provided by the Azure Kinect Sensor SDK. The sensor SDK automatically corrects for IMU intrinsics before sensor data is emitted.
- `body_tracking_data` (`visualization_msgs::MarkerArray`) : Topic for receiving body tracking data. Each message contains all joints for all bodies for the most recent depth image. The markers are grouped by the [body id](https://microsoft.github.io/Azure-Kinect-Body-Tracking/release/0.9.x/structk4abt__body__t.html#a38fed6c7125f92b41165ffe2e1da9dd4) and the joints are in the same order as in the [joint enum of body tracking sdk](https://microsoft.github.io/Azure-Kinect-Body-Tracking/release/0.9.x/group__btenums.html#ga5fe6fa921525a37dec7175c91c473781). The id field of a marker is calculated by: `body_id * 100 + joint_index` where body_id is the corresponding body id from the body tracking sdk and the joint index is analogue to enum value for joints in the body tracking sdk. Subscribers can calculate the body.id by `floor(marker.id / 100)` and the joint_index by `marker.id % 100`.
- `body_index_map/image_raw` (`sensor_msgs::Image`) : The [body index map](https://docs.microsoft.com/de-de/azure/Kinect-dk/body-index-map) represented as mono8 image with a background value of 255.
Up until body id 254 the pixel values of detected bodies will be equal to their corresponding body id. Afterwards the pixel value of detected bodies will be calculated as `body_id % 255` and therefore can only be used for segmentation without a relation to the body id.

## Azure Kinect Developer Kit Calibration

Unlike previous ROS Kinect drivers, the Azure Kinect ROS Driver provides calibration data from the sensor, decoded from the Azure Kinect DK native data format. This calibration information is used to populate several important pieces of ROS calibration information.

- tf2 : The relative offsets of the cameras and IMU are loaded from the Azure Kinect DK extrinsics calibration information. The various co-ordinate frames published by the node to TF2 are based on the calibration data. Please note that the calibration information only provides the relative positions of the IMU, color and depth cameras. It does not provide the offset between the sensors and the mechanical housing of the camera. The transform between the `camera_base` frame and `depth_camera_link` frame are based on the mechanical specifications of the Azure Kinect DK, and are not corrected by calibration.
- sensor_msgs::CameraInfo : Intrinsics calibration data for the cameras is converted into a ROS-compatible format. Fully populated CameraInfo messages are published for both the depth and color cameras, allowing ROS to undistort the images using image_proc and project them into point clouds using depth_image_proc. Using the point cloud functions in this node will provide GPU-accelerated results, but the same quality point clouds can be produced using standard ROS tools.

The driver integrates the `camera_info_manager` and provides the `set_camera_info` service to override the factory RGB and IR intrinsics in the ROS node. This is useful for a [manual calibration](http://wiki.ros.org/camera_calibration). Note that this will not override the factory calibration stored on the device.