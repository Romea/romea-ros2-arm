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
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import Node, PushRosNamespace


from romea_common_bringup import device_namespace
from romea_arm_bringup import ArmMetaDescription, arm_prefix, create_configuration_file


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_robot_urdf_description(context):
    return LaunchConfiguration("robot_urdf_description").perform(context)


def get_meta_description(context):
    arm_meta_description_file_path = LaunchConfiguration(
        "meta_description_file_path"
    ).perform(context)

    return ArmMetaDescription(arm_meta_description_file_path)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    meta_description = get_meta_description(context)
    controller_manager_name = arm_prefix(robot_namespace, meta_description)+"controller_manager"
    arm_name = meta_description.get_name()

    # controller_manager_configuration_file_path = create_configuration_file(
    #     robot_namespace, meta_description, "controller_manager"
    # )

    controllers_configuration_file_path = create_configuration_file(
        robot_namespace, meta_description, "controllers"
    )

    actions = []

    actions.append(PushRosNamespace(robot_namespace))
    actions.append(PushRosNamespace(arm_name))

    if get_mode(context) == "live":
        pass
        # ros2_control_config_urdf_file = "/tmp/"+prefix+name+"_ros2_control.urdf"
        # with open(base_ros2_control_description_file, "r") as f:
        # base_ros2_control_description = f.read()

        # if meta_description.get_type != "ur":
        #     actions.append(
        #         Node(
        #             package="controller_manager",
        #             executable="ros2_control_node",
        #             parameters=[
        #                 {"robot_description": robot_urdf_description},
        #                 controller_manager_configuration_file_path,
        #             ],
        #             # output="screen",
        #         )
        #     )
        # else:
        #     pass

    actions.append(
        Node(
            package="romea_mobile_base_controllers",
            # package="controller_manager",
            executable="spawner.py",
            exec_name="joint_state_broadcaster_spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                controller_manager_name,
            ],
            # output="screen",
        )
    )

    for controller_name in meta_description.get_controllers()["selected"]:

        actions.append(
            Node(
                package="romea_mobile_base_controllers",
                # package="controller_manager",
                executable="spawner.py",
                exec_name=controller_name + "_spawner",
                arguments=[
                    "joint_trajectory_controller",
                    "--param-file",
                    controllers_configuration_file_path,
                    "--controller-manager",
                    controller_manager_name,
                    "--namespace",
                    device_namespace(robot_namespace, None, arm_name)
                ],
                # output="screen",
            )
        )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("meta_description_file_path"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
