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

import os

from ament_index_python.packages import get_package_share_directory

import romea_arm_description
from romea_common_meta_bringup.meta_description import SensorMetaDescription
from romea_common_meta_bringup.ros_launch import LaunchFileGenerator
from romea_common_meta_bringup.utils import get_ros_distro
from romea_common_utils import render_template_file


class ArmMetaDescription(SensorMetaDescription):
    def __init__(self, meta_description_file_path, robot_name=None):
        self.__file_path = meta_description_file_path
        super().__init__("arm", meta_description_file_path, robot_name)

    def get_control_rate(self):
        return self._get_or("control_rate", "configuration", None)

    def get_home_joint_positions(self):
        return self._get_or("home_joints_positions", "configuration", None)

    def get_file_path(self):
        return self.__file_path


def load_meta_description(meta_description_file_path, robot_name=None):
    return ArmMetaDescription(meta_description_file_path, robot_name)


def get_specifications(meta_description):
    return romea_arm_description.get_specifications(meta_description.get_configuration())


def get_complete_configuration(meta_description):
    return romea_arm_description.get_complete_configuration(
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_location(),
    )


def generate_yaml_configuration_file_str(meta_description, extended):
    configuration = get_complete_configuration(meta_description)
    return romea_arm_description.generate_configuration_file_str(configuration, extended)


def get_template_controller_manager_configuration_file_path(mode, meta_description):
    manufacturer = meta_description.get_manufacturer()
    pkg = get_package_share_directory("romea_arm_meta_bringup")
    file_path = f"{pkg}/config/{manufacturer}/controller_manager_{mode}.yaml"

    if not os.path.exists(file_path):
        raise FileNotFoundError(
            f"Controller manager configuration for '{manufacturer}' arm not found: {file_path}"
        )

    return file_path


def get_template_controller_configurations_file_path(mode, meta_description):
    manufacturer = meta_description.get_manufacturer()
    pkg = get_package_share_directory("romea_arm_meta_bringup")
    file_path = f"{pkg}/config/{manufacturer}/controller_configurations_{mode}.yaml"

    if not os.path.exists(file_path):
        raise FileNotFoundError(
            f"Controller configuration for '{manufacturer}' arm not found: {file_path}"
        )

    return file_path


# TODO a factoriser avec implement
def get_template_controllers_configuration_file_path(mode, meta_description):
    manufacturer = meta_description.get_manufacturer()
    short_mode = "simulation" if "simulation" in mode else mode
    pkg = get_package_share_directory("romea_arm_meta_bringup")
    file_path = f"{pkg}/config/{manufacturer}_controllers_{short_mode}.yaml"

    if not os.path.exists(file_path):
        raise FileNotFoundError(
            f"Controllers configuration for '{manufacturer}' arm not found: {file_path}"
        )

    return file_path


def generate_yaml_controller_manager_configuration_file_str(mode, meta_description):
    file = get_template_controller_manager_configuration_file_path(mode, meta_description)

    context = {
        "tf_prefix": f"{meta_description.get_urdf_prefix()}{meta_description.get_name()}_",
        "controller_prefix": "",
        "control_rate": get_complete_configuration(meta_description)["control_rate"],
        "ros_namespace": meta_description.get_full_namespace(),
    }

    return render_template_file(file, context)


def generate_yaml_controller_configurations_file_str(mode, meta_description):
    file = get_template_controller_configurations_file_path(mode, meta_description)

    context = {
        "tf_prefix": f"{meta_description.get_urdf_prefix()}{meta_description.get_name()}_",
        "controller_prefix": "",
        "control_rate": get_complete_configuration(meta_description)["control_rate"],
        "ros_namespace": meta_description.get_full_namespace(),
    }

    return render_template_file(file, context)


def generate_yaml_controllers_configuration_file_str(mode, meta_description):
    context = {
        "tf_prefix": f"{meta_description.get_urdf_prefix()}{meta_description.get_name()}_",
        "controller_prefix": "",
        "control_rate": get_complete_configuration(meta_description)["control_rate"],
        "ros_namespace": meta_description.get_full_namespace(),
    }

    return render_template_file(
        get_template_controllers_configuration_file_path(mode, meta_description), context
    )


def generate_yaml_launch_file_str(meta_description):
    launch_file = meta_description.get_launch_file()
    pkg = "$(find-pkg-share romea_arm_meta_bringup)"
    manufacturer = meta_description.get_manufacturer()
    launch_arguments = [
        {"name": "mode", "default": "live"},
        {
            "name": "ros2_control_description",
            "default": (
                "$(command '"
                "generate-arm-urdf-description "
                "mode:=$(var mode) "
                f"robot_namespace:={meta_description.get_robot_name()} "
                f"meta_description_file_path:={meta_description.get_file_path()} "
                "standalone:=true "
                "generate_ros2_control_tag:=true"
                "' ignore)"
            ),
        },
        {
            "name": "controller_configurations_file_path",
            "default": f"{pkg}/config/{manufacturer}/controller_configurations_$(var mode).yaml",
        },
    ]

    namespaces = [
        meta_description.get_robot_name(),
        meta_description.get_namespace(),
        meta_description.get_name(),
    ]
    configuration = get_complete_configuration(meta_description)
    configuration["tf_prefix"] = (
        f"{meta_description.get_urdf_prefix()}{meta_description.get_name()}_"
    )
    configuration["frame_id"] = meta_description.get_link()

    return LaunchFileGenerator("arm").generate(
        launch_file, launch_arguments, namespaces, configuration
    )


def generate_xml_urdf_description_str(mode, meta_description, additional_urdf_arguments=None):
    is_gazebo_mode = mode == "simulation" or "gazebo" in mode
    additional_urdf_arguments = {
        "standalone": "false",
        "ros_distro": get_ros_distro(),
        "generate_gazebo_tag": "true" if is_gazebo_mode else "false",
        "generate_ros2_control_tag": "false",
        "controllers_config_yaml_file": None,
        "controller_manager_configuration_file_path": None,
        **dict(additional_urdf_arguments or {}),
    }

    if additional_urdf_arguments["generate_gazebo_tag"] == "true":
        if additional_urdf_arguments.get("controller_configurations_file_path") is None:
            additional_urdf_arguments["controllers_config_yaml_file"] = (
                get_template_controller_manager_configuration_file_path(mode, meta_description)
            )

    return romea_arm_description.generate_urdf_description_str(
        meta_description.get_urdf_prefix(),
        mode,
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_location(),
        meta_description.get_full_namespace(),
        additional_urdf_arguments,
    )
