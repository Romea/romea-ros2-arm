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

import romea_common_description
from romea_common_utils import render_template_file, save_temporary_file

import yaml


def get_specifications_file_path(arm_description):
    return romea_common_description.get_specifications_file_path(
        "romea_arm_description", arm_description
    )


def get_specifications(arm_description):
    with open(get_specifications_file_path(arm_description)) as f:
        return yaml.safe_load(f)


# def get_geometry_file_path(imu_description):
#     return romea_common_description.get_geometry_file_path(
#         "romea_imu_description", imu_description)


# def get_geometry(imu_description):
#     with open(get_geometry_file_path(imu_description)) as f:
#         return yaml.safe_load(f)


def get_specification_units_file_path():
    pkg_path = get_package_share_directory("romea_arm_description")
    return f"{pkg_path}/config/specifications_units.yaml"


def get_specification_units():
    with open(get_specification_units_file_path()) as f:
        return yaml.safe_load(f)


def get_xacro_file_path(arm_description):
    pkg = get_package_share_directory("romea_arm_description")
    return f"{pkg}/urdf/{arm_description["manufacturer"]}.xacro.urdf"


def get_complete_configuration(arm_name, arm_description, arm_location):

    model = arm_description["model"]
    version = arm_description["version"]
    manufacturer = arm_description["manufacturer"]
    arm_name = f"{manufacturer} {model} {version} arm called {arm_name}"
    specifications = get_specifications(arm_description)
    specifications_units = get_specification_units()

    arm = romea_common_description.DeviceConfiguration(
        arm_name, specifications, arm_description, specifications_units
    )

    configuration = {}
    configuration["model"] = arm_description["model"]
    configuration["version"] = arm_description["version"]
    configuration["manufacturer"] = arm_description["manufacturer"]
    configuration["control_rate"] = arm.get("control_rate")
    configuration["home_joint_positions"] = arm.get("home_joint_positions")
    return {**configuration, **arm_location}


def generate_configuration_file_str(configuration, extended=False):
    units = get_specification_units()
    return romea_common_description.generate_configuration_file(configuration, units, extended)


def generate_controllers_configuration_file_str(
    user_filename, prefix, arm_name, configuration, ros_namespace
):
    with open(user_filename, "r") as f:
        content = f.read()

    content = content.replace("$(var tf_prefix)", prefix + arm_name + "_")
    content = content.replace("$(var control_rate)", str(configuration["control_rate"]))
    content = content.replace("$(var ros_namespace)", ros_namespace)
    return content


def generate_urdf_description_str(
    prefix,
    mode,
    arm_name,
    arm_description,
    arm_location,
    ros_namespace,
    additional_urdf_arguments={},
):

    if mode == "simulation":
        mode += "_gazebo"

    configuration = get_complete_configuration(arm_name, arm_description, arm_location)

    configuration_yaml_file = save_temporary_file(
        f"{prefix}{arm_name}_configuration.yaml",
        generate_configuration_file_str(configuration)
    )

    additional_urdf_arguments = dict(additional_urdf_arguments or {})

    if "gazebo" in mode and additional_urdf_arguments.get("generate_gazebo_tag", "true") == "true":

        controllers_config_file_path = additional_urdf_arguments["controllers_config_yaml_file"]

        context = {
            "tf_prefix": f"{prefix}{arm_name}_",
            "control_rate": configuration["control_rate"],
            "ros_namespace": ros_namespace,
        }

        additional_urdf_arguments["controllers_config_yaml_file"] = save_temporary_file(
            f"{prefix}{arm_name}_controllers_configuration.yaml",
            render_template_file(controllers_config_file_path, context)
        )

    return romea_common_description.generate_urdf_description_str(
        get_xacro_file_path(arm_description),
        mappings={
            "tf_prefix": prefix,
            "mode": mode,
            "name": arm_name,
            "ros_namespace": ros_namespace,
            "arm_config_yaml_file": configuration_yaml_file,
            **additional_urdf_arguments,
        },
    )
