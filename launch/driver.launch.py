# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments ...
        launch.actions.DeclareLaunchArgument(
            'depth_enabled', 
            default_value="true", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'depth_mode', 
            default_value="WFOV_UNBINNED", 
            description="Set the depth camera mode, which affects FOV, depth range, and camera resolution. See Azure Kinect documentation for full details. Valid options: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR"
        ),
        launch.actions.DeclareLaunchArgument(
            'color_enabled', 
            default_value="true", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'color_format', 
            default_value="bgra", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'color_resolution', 
            default_value="1536P", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'fps', 
            default_value="5", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'point_cloud', 
            default_value="true", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'rgb_point_cloud', 
            default_value="true", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'point_cloud_in_depth_frame', 
            default_value="false", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'required', 
            default_value="false", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'sensor_sn', 
            default_value="", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'recording_file', 
            default_value="", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'recording_loop_enabled', 
            default_value="false", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'body_tracking_enabled', 
            default_value="false", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'body_tracking_smoothing_factor', 
            default_value="0.0", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'rescale_ir_to_mono8', 
            default_value="false", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'ir_mono8_scaling_factor', 
            default_value="1.0", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'imu_rate_target', 
            default_value="0", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'wired_sync_mode', 
            default_value="0", 
            description="Enable or disable the depth camera"
        ),
        launch.actions.DeclareLaunchArgument(
            'subordinate_delay_off_master_usec', 
            default_value="0", 
            description="Enable or disable the depth camera"
        ),
        # Launch node ...
        launch_ros.actions.Node(
            package='azure_kinect_ros_driver',
            namespace='azure_kinect_ros_driver_node',
            executable='node',
            name='azure_kinect_ros_driver',
            parameters=[
                {"depth_enabled": launch.substitutions.LaunchConfiguration('depth_enabled')}
            ]
        )
    ])