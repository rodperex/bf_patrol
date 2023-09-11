# Copyright 2023 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # Get the launch directory
    sp_dir = get_package_share_directory('bf_patrol')

    params = os.path.join(
        get_package_share_directory('bf_patrol'),
        'params',
        'remote_config.yaml'
    )
  
    remote_cmd_1 = Node(
        package='bf_patrol',
        executable='single_remote',
        # name='R1',
        namespace='robot1',
        output='screen',
        parameters=[params],
        arguments=['R1'],
        remappings=[
            ('input_scan', '/scan'),
            ('output_vel', '/cmd_vel')],
        )
    remote_cmd_2 = Node(
        package='bf_patrol',
        executable='single_remote',
        # name='R2',
        namespace='robot2',
        output='screen',
        parameters=[params],
        arguments=['R2'],
        remappings=[
            ('input_scan', '/scan'),
            ('output_vel', '/cmd_vel')],
        # TimerAction(period=1.0)
        )
    remote_cmd_3 = Node(
        package='bf_patrol',
        executable='single_remote',
        # name='R3',
        namespace='robot3',
        output='screen',
        parameters=[params],
        arguments=['R3'],
        remappings=[
            ('input_scan', '/scan'),
            ('output_vel', '/cmd_vel')]
        )  

    remote_cmd_4 = Node(
        package='bf_patrol',
        executable='single_remote',
        # name='R4',
        namespace='robot4',
        output='screen',
        parameters=[params],
        arguments=['R4'],
        remappings=[
            ('input_scan', '/scan'),
            ('output_vel', '/cmd_vel')],
        # prefix=['xterm -e gdb -ex run --args']

        )  


    # Create the launch description and populate
    # ld = LaunchDescription()

    # ld.add_action(remote_cmd_1)
    # ld.add_action(remote_cmd_2)
    # ld.add_action(remote_cmd_3)

    # return ld

    return LaunchDescription([
        TimerAction(
            period=0.0,
            actions=[remote_cmd_1]
        ),
        TimerAction(
            period=3.0,
            actions=[remote_cmd_2]
        ),
        TimerAction(
            period=5.0,
            actions=[remote_cmd_3]
        ),
        TimerAction(
            period=7.0,
            actions=[remote_cmd_4]
        )
    ])
