#!/usr/bin/env python3
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    camera_params = LaunchConfiguration(
        'camera_params',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            'camera.yaml'))

    # camera_ros expects the calibration file to be in the ~/.ros folder
    # with a particular name. It's hacky, but we just copy it there.
    calibration_file_path = os.path.join(get_package_share_directory('turtlebot3_bringup'),
                                         'camera_info',
                                         'turtlebot3_rpicamera.yaml')

    calibration_target_path = os.path.join(os.path.expanduser('~'),
                                           '.ros', 'camera_info',
                                           'imx219__base_soc_i2c0mux_i2c_1_imx219_10_640x360.yaml')
    if not os.path.exists(calibration_target_path):
        os.makedirs(os.path.dirname(calibration_target_path), exist_ok=True)
        shutil.copy(calibration_file_path, calibration_target_path)

    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            parameters=[camera_params],
            # Remap image_raw so that it can be created from compressed
            # images on the workstation side.
            remappings=[
                ('/camera/image_raw', '/camera/image_raw/uncompressed'),
            ],
            output='screen'
            ),

        Node(
            package='topic_tools',
            executable='throttle',
            parameters = [{'input_topic': '/camera/image_raw/compressed',
                          'msgs_per_sec': 5.0,
                          'throttle_type': 'messages'}],
            arguments=['messages'],
            output='screen'
            ),

          Node(
              package = "tf2_ros", 
              executable = "static_transform_publisher",
              arguments = ["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "camera_rgb_optical_frame", "camera"]),



    ])
