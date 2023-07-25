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
from launch.actions import IncludeLaunchDescription
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
  
    remote_cmd = Node(
        package='bf_patrol',
        executable='single_remote',
        # name='remote',
        namespace='robot3',
        output='screen',
        parameters=[params],
        arguments=['patrol_config.yaml'],
        remappings=[
            ('input_scan', '/scan'),
            ('output_vel', '/cmd_vel')]
        )  

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(remote_cmd)
    
    return ld
