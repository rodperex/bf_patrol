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
    bf_patrol_dir = get_package_share_directory('bf_patrol')

    config = os.path.join(bf_patrol_dir, 'config', 'patrol_params.yaml')
    with open(config, "r") as stream:
        try:
            conf = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    yaml_file = [conf['bf_patrol']['waypoints'] + '.yaml']
    print(yaml_file)
   
    ld = LaunchDescription()

    robot_cmd = Node(
        package='bf_patrol',
        executable='patrol',
        output='screen',
        arguments=yaml_file,
        parameters=[{'use_sim_time': False}]
    )

    ld.add_action(robot_cmd)

    return ld

