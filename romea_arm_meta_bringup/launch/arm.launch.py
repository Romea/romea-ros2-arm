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

# from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource

from romea_arm_meta_bringup.meta_description import (
    generate_yaml_launch_file_str,
)
import romea_arm_meta_bringup.ros_launch as arm
import romea_common_meta_bringup.ros_launch as common


def launch_setup(context, *args, **kwargs):
    mode = common.get_mode(context)
    meta_description = arm.get_meta_description(context)

    launch_filename = f"/tmp/{meta_description.get_filename_prefix()}drivers.launch.yaml"
    with open(launch_filename, "w") as f:
        f.write(generate_yaml_launch_file_str(meta_description))

    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_filename),
            launch_arguments={
                "mode": mode,
            }.items(),
        )
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            common.declare_mode("live"),
            common.declare_robot_namespace(""),
            common.declare_meta_description_file_path("arm"),
            OpaqueFunction(function=launch_setup),
        ]
    )
