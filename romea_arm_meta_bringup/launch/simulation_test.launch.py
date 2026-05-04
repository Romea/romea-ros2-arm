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


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import romea_common_meta_bringup.ros_launch as common
import romea_simulation_meta_bringup.ros_launch as simulation


def launch_setup(context, *args, **kwargs):

    robot_namespace = common.get_robot_namespace(context)
    simulator_type = simulation.get_simulator_type(context)
    meta_description_file_path = common.get_meta_description_file_path(context)

    launch = LaunchDescription()

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulator.launch.py"
        ),
        launch_arguments={'simulator_type': simulator_type}.items(),
    )

    launch.add_action(simulator)

    entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/entity.launch.py"
        ),
        launch_arguments={
            'simulator_type': simulator_type,
            'entity_type': "arm",
            'robot_namespace': robot_namespace,
            'meta_description_file_path': meta_description_file_path,
        }.items(),
    )

    launch.add_action(entity)

    nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_arm_meta_bringup")
            + "/launch/arm.launch.py"
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'mode': f"simulation_{simulator_type}",
            'meta_description_file_path': meta_description_file_path,
        }.items(),
    )

    launch.add_action(nodes)

    return [launch]


def generate_launch_description():

    return LaunchDescription(
        [
           common.declare_robot_namespace("robot"),
           simulation.declare_simulator_type("gazebo"),
           common.declare_meta_description_file_path("arm"),
           OpaqueFunction(function=launch_setup)
        ]
    )
