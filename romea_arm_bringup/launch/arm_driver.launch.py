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

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import PushRosNamespace
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_common_bringup import device_link_name
from romea_arm_bringup import ArmMetaDescription


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description(context):
    arm_meta_description_file_path = LaunchConfiguration(
        "meta_description_file_path"
    ).perform(context)

    return ArmMetaDescription(arm_meta_description_file_path)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)

    if not meta_description.has_driver_configuration:
        return []

    gps_name = meta_description.get_name()
    gps_namespace = str(meta_description.get_namespace() or "")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_arm_bringup"),
                        "launch",
                        "drivers/" + meta_description.get_driver_pkg() + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "device": meta_description.get_driver_device(),
            "baudrate": str(meta_description.get_driver_baudrate()),
            "rate": str(meta_description.get_rate()),
            "frame_id": device_link_name(robot_namespace, gps_name),
        }.items(),
    )

    actions = [
        PushRosNamespace(robot_namespace),
        PushRosNamespace(gps_namespace),
        PushRosNamespace(gps_name),
        driver,
    ]

    if meta_description.has_ntrip_configuration():

        ntrip = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("romea_gps_bringup"),
                            "launch",
                            "drivers/"
                            + meta_description.get_ntrip_pkg()
                            + ".launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "host": meta_description.get_ntrip_host(),
                "port": str(meta_description.get_ntrip_port()),
                "mountpoint": meta_description.get_ntrip_mountpoint(),
                "username": meta_description.get_ntrip_username(),
                "password": meta_description.get_ntrip_password(),
            }.items(),
        )

        actions.append(ntrip)

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
