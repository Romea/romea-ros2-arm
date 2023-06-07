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
)

from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_meta_description_file_path(context):
    return LaunchConfiguration("meta_description_file_path").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    meta_description_file_path = get_meta_description_file_path(context)

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_arm_bringup"),
                        "launch",
                        "arm_driver.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "meta_description_file_path": meta_description_file_path,
        }.items(),
        condition=LaunchConfigurationEquals("mode", "live"),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_arm_bringup"),
                        "launch",
                        "arm_controllers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "mode": mode,
            "robot_namespace": robot_namespace,
            "meta_description_file_path": meta_description_file_path,
        }.items(),
    )

    return [driver, controllers]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace", default_value="")
    )

    declared_arguments.append(
        DeclareLaunchArgument("mode", default_value="live")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
