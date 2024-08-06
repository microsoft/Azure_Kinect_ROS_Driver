import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
import os

def generate_launch_description():
    azure_kinect_ros_driver_node = launch_ros.actions.Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        parameters=[
            {'depth_enabled': True},
            {'depth_mode': 'NFOV_2X2BINNED'},
            {'depth_unit': '16UC1'},
            {'color_enabled': True},
            {'color_format': 'bgra'},
            {'color_resolution': '1080P'},
            {'fps': 30},
            {'point_cloud': True},
            {'rgb_point_cloud': True},
            {'point_cloud_in_depth_frame': True},
            {'imu_rate_target': 0},
            {'exposure_auto': True},
            {'exposure_time': 15000},
            {'whitebalance_auto': True},
            {'whitebalance_val': 4500},
            {'brightness': 128},
            {'contrast': 5},
            {'saturation': 32},
            {'sharpness': 2},
            {'gain': 128}
        ]
    )
    azure_kinect_debugger_node = launch_ros.actions.Node(
        package='azure_kinect_debugger',
        executable='kinect_subscriber'    
    )

    return launch.LaunchDescription([
        azure_kinect_ros_driver_node,
        azure_kinect_debugger_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
