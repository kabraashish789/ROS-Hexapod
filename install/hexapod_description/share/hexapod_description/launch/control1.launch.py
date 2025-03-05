from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            ),
       Node(
          package='hexapod_description',
          executable='motor_data1',
          name='motor_data',
          parameters=[{'use_sim_time': True}],
          output='screen'),
    ])
  
'''return LaunchDescription([
        launch_ros.actions.Node(
            package='hexapod_description',
            executable='motor_data',
            output='screen',
            arguments=["0.5"]),
    ])'''