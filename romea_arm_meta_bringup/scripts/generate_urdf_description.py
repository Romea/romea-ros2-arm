#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import sys

from romea_arm_meta_bringup.meta_description import ArmMetaDescription
from romea_arm_meta_bringup.meta_description import (
    generate_xml_urdf_description_str,
    get_template_controllers_configuration_file_path
)
from romea_common_meta_bringup.script_parameters import (
    robot_urdf_description_generation_parameters_from_cli,
)


def main():

    parameters = robot_urdf_description_generation_parameters_from_cli("arm")
    mode = parameters.pop_str("mode", required=True)
    robot_namespace = parameters.pop_str("robot_namespace", required=True)
    meta_description_file_path = parameters.pop_str("meta_description_file_path", required=True)
    meta_description = ArmMetaDescription(meta_description_file_path, robot_namespace)

    additional_arguments = {
        "standalone": parameters.pop_bool("standalone"),
        "ros_distro": parameters.pop_str("ros_distro"),
        "generate_ros2_control_tag": parameters.pop_bool("generate_ros2_control_tag"),
        "generate_gazebo_tag": (
            parameters.pop_bool("generate_gazebo_tag")
            if mode == "simulation" or "gazebo" in mode
            else "false"
        ),
    }

    controllers_config_yaml_file = parameters.pop_str(
        "controller_configuration_file_path",
        get_template_controllers_configuration_file_path(mode, meta_description)
    )

    if additional_arguments["generate_gazebo_tag"] == "true":
        additional_arguments["controllers_config_yaml_file"] = controllers_config_yaml_file

    additional_arguments.update(parameters.remaining())

    print(generate_xml_urdf_description_str(mode, meta_description, additional_arguments))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
