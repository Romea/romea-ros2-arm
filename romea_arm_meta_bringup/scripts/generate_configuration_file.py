#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK

import sys

from romea_arm_meta_bringup.meta_description import ArmMetaDescription
from romea_arm_meta_bringup.meta_description import generate_yaml_configuration_file_str
from romea_common_meta_bringup.script_parameters import (
    configuration_file_generation_parameters_from_cli,
)


def main():
    parameters = configuration_file_generation_parameters_from_cli("arm")
    extended = parameters.pop_bool("extended") == "true"
    meta_description_file_path = parameters.pop_str("meta_description_file_path", required=True)
    meta_description = ArmMetaDescription(meta_description_file_path)
    print(generate_yaml_configuration_file_str(meta_description, extended))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
