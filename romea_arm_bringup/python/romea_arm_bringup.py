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

from romea_common_bringup import (
    MetaDescription,
    robot_urdf_prefix,
    robot_prefix,
    device_urdf_prefix,
    device_prefix,
    get_file_path,
)
import romea_arm_description
import yaml


class ArmMetaDescription:
    def __init__(self, meta_description_file_path):
        self.meta_description = MetaDescription("arm", meta_description_file_path)

    def get_name(self):
        return self.meta_description.get("name")

    def get_namespace(self):
        return self.meta_description.get_or("namespace", None)

    def has_driver_configuration(self):
        return self.meta_description.exists("driver")

    def get_driver_pkg(self):
        return self.meta_description.get("pkg", "driver")

    def get_driver_ip(self):
        return self.meta_description.get("ip", "driver")

    def get_driver_port(self):
        return self.meta_description.get("port", "driver")

    def get_controller_manager(self):
        return self.meta_description.get("controller_manager", "control")

    def get_controllers(self):
        return self.meta_description.get("controllers", "control")

    def get_type(self):
        return self.meta_description.get("type", "configuration")

    def get_model(self):
        return self.meta_description.get("model", "configuration")

    def get_calibration(self):
        return self.meta_description.get("calibration", "configuration")

    def get_joint_limits(self):
        return self.meta_description.get("joint_limits", "configuration")

    def get_initial_joint_positions(self):
        return self.meta_description.get("initial_joint_positions", "configuration")

    def get_parent_link(self):
        return self.meta_description.get("parent_link", "geometry")

    def get_xyz(self):
        return self.meta_description.get("xyz", "geometry")

    def get_rpy(self):
        return self.meta_description.get("rpy", "geometry")


def get_control_configuration_file_path(
    control_meta_description, default_configuration_file
):
    if "configuration" in control_meta_description:
        return get_file_path(control_meta_description["configuration"])
    else:
        return get_file_path(
            {"pkg": "romea_ur_bringup", "file": default_configuration_file}
        )


def get_control_configuration(control_meta_description, default_configuration_file):

    configuration_file_path = get_control_configuration_file_path(
        control_meta_description, default_configuration_file
    )

    with open(configuration_file_path) as f:
        return yaml.safe_load(f)


def arm_prefix(robot_namespace, meta_description):
    return device_prefix(
        robot_namespace, meta_description.get_namespace(), meta_description.get_name()
    )


def arm_urdf_prefix(robot_namespace, meta_description):
    return device_urdf_prefix(robot_namespace, meta_description.get_name())


def get_controllers_configuration(
    robot_namespace, meta_description, default_configuration_file
):
    controllers = meta_description.get_controllers()
    configuration = get_control_configuration(controllers, default_configuration_file)

    urdf_prefix = arm_urdf_prefix(robot_namespace, meta_description)

    for controller_name in configuration.keys():
        parameters = configuration[controller_name]["ros__parameters"]
        joints = parameters.get("joints", None)
        constraints = parameters.get("constraints", None)

        if joints is not None:
            if constraints is not None:
                for joint in joints:
                    constraints[urdf_prefix+joint] = constraints.pop(joint)
            parameters["joints"] = [urdf_prefix + joint for joint in joints]

    ros_prefix = arm_prefix(robot_namespace, meta_description)
    return {ros_prefix + k: v for k, v in configuration.items()}


def get_controller_manager_configuration(
    robot_namespace, meta_description, default_configuration_file
):
    controller_manager = meta_description.get_controller_manager()
    configuration = get_control_configuration(
        controller_manager, default_configuration_file
    )

    parameters = configuration["controller_manager"]["ros__parameters"]
    parameters["update_rate"] = controller_manager["update_rate"]

    ros_prefix = arm_prefix(robot_namespace, meta_description)
    return {ros_prefix + k: v for k, v in configuration.items()}


def create_configuration_file(robot_namespace, meta_description, what):

    configuration = globals()["get_" + what + "_configuration"](
        robot_namespace,
        meta_description,
        meta_description.get_type() + "_" + what + ".yaml",
    )

    configuration_file_path = (
        "/tmp/"
        + robot_namespace
        + "_"
        + meta_description.get_name()
        + "_"
        + what
        + ".yaml"
    )

    with open(configuration_file_path, "w") as f:
        yaml.dump(configuration, f)

    return configuration_file_path


def urdf_description(robot_namespace, mode, meta_description_file_path):

    meta_description = ArmMetaDescription(meta_description_file_path)

    controller_manager_configuration_file_path = create_configuration_file(
        robot_namespace, meta_description, "controller_manager"
    )

    return romea_arm_description.ur_arm_urdf(
        robot_urdf_prefix(robot_namespace),
        mode,
        meta_description.get_name(),
        meta_description.get_model(),
        meta_description.get_parent_link(),
        meta_description.get_xyz(),
        meta_description.get_rpy(),
        meta_description.get_driver_ip(),
        get_file_path(meta_description.get_calibration()),
        get_file_path(meta_description.get_joint_limits()),
        get_file_path(meta_description.get_initial_joint_positions()),
        controller_manager_configuration_file_path,
        robot_prefix(robot_namespace),
    )
