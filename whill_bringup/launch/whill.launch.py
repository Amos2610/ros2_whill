# Copyright (c) 2024 WHILL, Inc.
# Released under the MIT license
# https://opensource.org/licenses/mit-license.php

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    # model_file =  + "whill_description/urdf/whill_model_cr2.urdf"
    model_file = os.path.join(get_package_share_directory('whill_description'), 'urdf', 'whill_model_cr2.urdf')
    print(model_file)
    serial_port = LaunchConfiguration('serialport', default='$(env TTY_WHILL)')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(model_file, 'r').read()}],
            remappings=[('/joint_states', '/whill/joint_states')]
        ),
        Node(
            package='whill_driver',
            namespace='',
            executable='whill',
            name='whill',
            # load params.yaml
            parameters=[os.path.join(get_package_share_directory('whill_bringup'), 'params.yaml')],
            # remmaping
            remappings=[
                ('cmd_vel_nav', 'whill/controller/cmd_vel')
            ],
        ),

        # # Added by fumoto (whill_odometry)
        # Node(
        #     package='whill_driver',
        #     namespace='',
        #     executable='odom',  # odom ノードの実行可能ファイル
        #     name='whill_odometry',  # ノード名
        #     remappings=[
        #         ('odom', '/whill/odom')  # 必要ならトピック名をリマッピング
        #     ],
        # ),
    ])