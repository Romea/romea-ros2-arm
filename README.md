# romea_ros2_arm

## Overview

`romea_ros2_arm` groups the ROS2 packages used to describe and launch robotic arms in the ROMEA ecosystem.

This repository-level README gives a map of the stack. Detailed behavior, configuration formats, generated files and launch examples are documented in the README of each package listed below.

## Packages

| Package | Role |
| --- | --- |
| `romea_arm` | Metapackage for the arm stack. |
| `romea_arm_description` | Arm specifications, configuration files, Python helpers, controller configuration generation and URDF generation. |
| `romea_arm_meta_bringup` | Main user entry point: arm meta-description parser, configuration generation, controller configuration generation, URDF generation and reusable launch profiles. |

## Usage

This stack is usually consumed from a larger ROMEA workspace or from a demo configuration that already selects the arm devices to launch.

In most cases, start with `romea_arm_meta_bringup`. It is the user-facing entry point of the stack: from an arm meta-description, it can generate the detailed arm configuration, generate the ROS2 controller configuration, generate the URDF fragment, and launch the selected arm profile. `romea_arm_description` provides the arm description data, controller templates and URDF generation helpers used behind this entry point.

Use the specialized package README files when you need to inspect or extend a specific part of the stack:

* `romea_arm_description` to add or inspect arm specifications, controller templates and URDF generation;
* `romea_arm_meta_bringup` to write arm meta-descriptions and launch profiles.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `romea_ros2_arm` stack was developed by Jean Laneurit in the context of research projects carried out at INRAE.

## Contact

For questions or comments about this stack, contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
