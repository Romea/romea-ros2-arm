# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ros2_control_description = ParameterValue(
        LaunchConfiguration("ros2_control_description"),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="romea_common_meta_bringup",
                executable="urdf_broadcaster_node",
                name="ros2_control_description",
                parameters=[
                    {
                        "robot_description": ros2_control_description,
                    }
                ],
            ),
        ]
    )
