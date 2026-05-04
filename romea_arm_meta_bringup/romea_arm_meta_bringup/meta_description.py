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

import romea_arm_description
from romea_common_meta_bringup.meta_description import SensorMetaDescription
from romea_common_meta_bringup.ros_launch import LaunchFileGenerator
from romea_common_meta_bringup.utils import device_urdf_prefix

import yaml


class ArmMetaDescription(SensorMetaDescription):
    def __init__(self, meta_description_file_path, robot_name=None):
        super().__init__("arm", meta_description_file_path, robot_name)

    def get_control_rate(self):
        return self._get_or("control_rate", "configuration", None)

    def get_home_joint_positions(self):
        return self._get_or("home_joints_positions", "configuration", None)


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


def get_complete_controllers_configuration(mode, meta_description):
    string = generate_yaml_controllers_configuration_file_str(mode, meta_description)
    return yaml.safe_load(string)


def generate_yaml_controllers_configuration_file_str(mode, meta_description):
    manufacturer = meta_description.get_manufacturer()
    configuration = get_complete_configuration(meta_description)
    short_mode = "simulation" if "simulation" in mode else mode
    pkg = get_package_share_directory("romea_arm_meta_bringup")
    # return meta_description.generate_controllers_configuration_file(
    #     f"{pkg}/config/{manufacturer}_controllers_{short_mode}.yaml",
    #     meta_description.get_urdf_prefix(),
    #     meta_description.get_name(),
    #     configuration
    # )
    with open(f"{pkg}/config/{manufacturer}_controllers_{short_mode}.yaml", "r") as f:
        content = f.read()

    content = content.replace("$(var control_rate)", str(configuration["control_rate"]))
    content = content.replace("$(var ros_namespace)", meta_description.get_full_namespace())
    content = content.replace(
        "$(var tf_prefix)",
        device_urdf_prefix(meta_description.get_robot_name(), meta_description.get_name()),
    )
    return content


def generate_yaml_launch_file_str(meta_description):
    launch_file = meta_description.get_launch_file()
    pkg = "$(find-pkg-share romea_arm_meta_bringup)"
    manufacturer = meta_description.get_manufacturer()
    launch_arguments = [
        {"name": "mode", "default": "live"},
        {
            "name": "controllers_configuration_file_path",
            "default": f"{pkg}/config/{manufacturer}_controllers_$(var mode).yaml",
        },
    ]
    namespaces = [
        meta_description.get_robot_name(),
        meta_description.get_namespace(),
        meta_description.get_name(),
    ]
    configuration = get_complete_configuration(meta_description)
    configuration["tf_prefix"] = meta_description.get_urdf_prefix()
    configuration["frame_id"] = meta_description.get_link()

    return LaunchFileGenerator("arm").generate(
        launch_file, launch_arguments, namespaces, configuration
    )


def generate_xml_urdf_description_str(mode, meta_description, additional_urdf_arguments):

    return romea_arm_description.generate_urdf_description_str(
        meta_description.get_urdf_prefix(),
        mode,
        meta_description.get_name(),
        meta_description.get_configuration(),
        meta_description.get_location(),
        meta_description.get_full_namespace(),
        additional_urdf_arguments,
    )
