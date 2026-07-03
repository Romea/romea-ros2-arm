#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import sys

from romea_arm_meta_bringup.meta_description import ArmMetaDescription
from romea_arm_meta_bringup.meta_description import (
    generate_xml_urdf_description_str,
)
from romea_common_meta_bringup.script_parameters import (
    robot_urdf_description_generation_parameters_from_cli,
    # urdf_generation_options,
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
        "generate_gazebo_tag": (
            parameters.pop_bool("generate_gazebo_tag")
            if mode == "simulation" or "gazebo" in mode
            else "false"
        ),
    }

    # additional_arguments = urdf_generation_options(parameters, mode, ros2_control=True)
    additional_arguments.update(parameters.remaining())

    print(generate_xml_urdf_description_str(mode, meta_description, additional_arguments))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
