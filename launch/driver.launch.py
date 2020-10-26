# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

from launch import LaunchDescription

import launch.actions
import launch_ros.actions

def generate_launch_description():

    return LaunchDescription([
    launch.actions.DeclareLaunchArgument(
        'tf_prefix', 
        default_value="", 
        description="Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty."),
    launch.actions.DeclareLaunchArgument(
        'overwrite_robot_description', 
        default_value="true" , 
        description="Flag to publish a standalone azure_description instead of the default robot_descrition parameter."),
    ##############################################
    launch.actions.DeclareLaunchArgument(
        'depth_enabled', 
        default_value="true", 
        description="Enable or disable the depth camera"),
    launch.actions.DeclareLaunchArgument(
        'depth_mode', 
        default_value="WFOV_UNBINNED", 
        description="Set the depth camera mode, which affects FOV, depth range, and camera resolution. See Azure Kinect documentation for full details. Valid options: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR"),
    launch.actions.DeclareLaunchArgument(
        'color_enabled', 
        default_value="true", 
        description="Enable or disable the color camera"),
    launch.actions.DeclareLaunchArgument(
        'color_format', 
        default_value="bgra", 
        description="The format of RGB camera. Valid options: bgra, jpeg"),
    launch.actions.DeclareLaunchArgument(
        'color_resolution', 
        default_value="1536P", 
        description="Resolution at which to run the color camera. Valid options: 720P, 1080P, 1440P, 1536P, 2160P, 3072P"),
    launch.actions.DeclareLaunchArgument(
        'fps', 
        default_value="5", 
        description="FPS to run both cameras at. Valid options are 5, 15, and 30"),
    launch.actions.DeclareLaunchArgument(
        'point_cloud', 
        default_value="true", 
        description="Generate a point cloud from depth data. Requires depth_enabled"),
    launch.actions.DeclareLaunchArgument(
        'rgb_point_cloud', 
        default_value="true", 
        description="Colorize the point cloud using the RBG camera. Requires color_enabled and depth_enabled"),
    launch.actions.DeclareLaunchArgument(
        'point_cloud_in_depth_frame', 
        default_value="false", 
        description="Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). Will either match the resolution of the depth camera (true) or the RGB camera (false)."),
    launch.actions.DeclareLaunchArgument( # Not a parameter of the node, rather a launch file parameter
        'required', 
        default_value="false", 
        description="Argument which specified if the entire launch file should terminate if the node dies"),
    launch.actions.DeclareLaunchArgument(
        'sensor_sn', 
        default_value="", 
        description="Sensor serial number. If none provided, the first sensor will be selected"),
    launch.actions.DeclareLaunchArgument(
        'recording_file', 
        default_value="", 
        description="Absolute path to a mkv recording file which will be used with the playback api instead of opening a device"),
    launch.actions.DeclareLaunchArgument(
        'recording_loop_enabled', 
        default_value="false", 
        description="If set to true the recording file will rewind the beginning once end of file is reached"),
    launch.actions.DeclareLaunchArgument(
        'body_tracking_enabled', 
        default_value="false", 
        description="If set to true the joint positions will be published as marker arrays"),
    launch.actions.DeclareLaunchArgument(
        'body_tracking_smoothing_factor', 
        default_value="0.0", 
        description="Set between 0 for no smoothing and 1 for full smoothing"),
    launch.actions.DeclareLaunchArgument(
        'rescale_ir_to_mono8', 
        default_value="false", 
        description="Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied."),
    launch.actions.DeclareLaunchArgument(
        'ir_mono8_scaling_factor', 
        default_value="1.0", 
        description="Scaling factor to apply when converting IR to mono8 (see rescale_ir_to_mono8). If using illumination, use the value 0.5-1. If using passive IR, use 10."),
    launch.actions.DeclareLaunchArgument(
        'imu_rate_target', 
        default_value="0", 
        description="Desired output rate of IMU messages. Set to 0 (default) for full rate (1.6 kHz)."),
    launch.actions.DeclareLaunchArgument(
        'wired_sync_mode', 
        default_value="0", 
        description="Wired sync mode. 0: OFF, 1: MASTER, 2: SUBORDINATE."),
    launch.actions.DeclareLaunchArgument(
        'subordinate_delay_off_master_usec', 
        default_value="0", 
        description="Delay subordinate camera off master camera by specified amount in usec."),
    launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        parameters=[
            {'depth_enabled': launch.substitutions.LaunchConfiguration('depth_enabled')},
            {'depth_mode': launch.substitutions.LaunchConfiguration('depth_mode')}, 
            {'color_enabled': launch.substitutions.LaunchConfiguration('color_enabled')},
            {'color_format': launch.substitutions.LaunchConfiguration('color_format')}, 
            {'color_resolution': launch.substitutions.LaunchConfiguration('color_resolution')}, 
            {'fps': launch.substitutions.LaunchConfiguration('fps')},
            {'point_cloud': launch.substitutions.LaunchConfiguration('point_cloud')},
            {'rgb_point_cloud': launch.substitutions.LaunchConfiguration('rgb_point_cloud')},
            {'point_cloud_in_depth_frame': launch.substitutions.LaunchConfiguration('point_cloud_in_depth_frame')}, 
            {'sensor_sn': launch.substitutions.LaunchConfiguration('sensor_sn')},
            {'tf_prefix': launch.substitutions.LaunchConfiguration('tf_prefix')},
            {'recording_file': launch.substitutions.LaunchConfiguration('recording_file')}, 
            {'recording_loop_enabled': launch.substitutions.LaunchConfiguration('recording_loop_enabled')},
            {'body_tracking_enabled': launch.substitutions.LaunchConfiguration('body_tracking_enabled')},
            {'body_tracking_smoothing_factor': launch.substitutions.LaunchConfiguration('body_tracking_smoothing_factor')},
            {'rescale_ir_to_mono8': launch.substitutions.LaunchConfiguration('rescale_ir_to_mono8')},
            {'ir_mono8_scaling_factor': launch.substitutions.LaunchConfiguration('ir_mono8_scaling_factor')},
            {'imu_rate_target': launch.substitutions.LaunchConfiguration('imu_rate_target')},
            {'wired_sync_mode': launch.substitutions.LaunchConfiguration('wired_sync_mode')},
            {'subordinate_delay_off_master_usec': launch.substitutions.LaunchConfiguration('subordinate_delay_off_master_usec')}]),
    ])