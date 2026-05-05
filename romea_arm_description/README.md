# romea_arm_description

## Overview

`romea_arm_description` provides URDF descriptions and configuration utilities for robotic arms using a **specification-based approach**.

It allows defining arm characteristics and generating consistent ROS 2 artifacts using the provided Python module, such as:

* configuration files
* controllers configuration files
* URDF description files

This package is designed to be used together with `romea_arm_meta_bringup`.

---

## Arm description concept

An arm is described using two inputs:

* **arm description**: defines hardware characteristics (manufacturer, model, control, joints)
* **arm location**: defines how the arm is attached to the robot (parent link, pose)

These inputs are combined with arm specifications to build a complete configuration used to generate ROS 2 artifacts.

---

### Example

```yaml
arm_description:
  manufacturer: ur
  model: "05"
  version: e
  control_rate: 500 # Hz
  home_joint_positions: 
    shoulder_pan_joint: 0.0 #°
    shoulder_lift_joint: -90.0 #°
    elbow_joint: 0.0 #°
    wrist_1_joint: -90.0 #°
    wrist_2_joint: 0.0 #°
    wrist_3_joint: 0.0 #°

arm_location:
  parent_link: base_link
  xyz: [1.0, 2.0, 3.0] #m
  rpy: [4.0, 5.0, 6.0] #°
```

---

### Notes

Arm specifications are defined in files located in the `config/` directory and follow the pattern `<manufacturer>_<model>.<version>_specifications.yaml` (e.g. `ur_05.e_specifications.yaml`). These specification files provide default values such as control rate and joint configuration, which can be overridden by user-defined values in `arm_description`. As a result, parameters like `control_rate` and `home_joint_positions` are optional.

The `xyz` and `rpy` fields define the pose of the arm relative to the parent link. The `xyz` values specify the translation (in meters), while `rpy` defines the orientation using roll, pitch, and yaw angles expressed in degrees for readability and ease of definition by users. These angles are automatically converted to radians internally by the scripts to comply with ROS and URDF conventions. Together, they describe how the arm is positioned and oriented within the robot frame.

---

## Python API

The package provides utilities to generate configuration, controllers configuration and URDF descriptions from an arm description.

---

### get_complete_configuration

Builds a complete arm configuration by combining:

* arm description
* arm location
* arm specifications

---

### generate_configuration_file_str

Generates the arm configuration file as a YAML string.

---

### generate_controllers_configuration_file_str

Generates a ROS 2 controllers configuration as a string, ready to be written to a YAML file.

* applies joint names, namespace and prefix
* configures control rate and interfaces

The generated file is intended for the **ROS2 controller manager**.

---

### generate_urdf_description_str

Generates the URDF description of the arm as a string, ready to be written to a URDF file.

* supports live and simulation modes
* integrates optional tags (e.g. `ros2_control`, simulation plugins)

The generated URDF is used by simulators, controllers and visualization tools.

---

## Example

```python
from romea_arm_description import (
    get_complete_configuration,
    generate_configuration_file_str,
    generate_controllers_configuration_file_str,
    generate_urdf_description_str,
)

arm_name = "arm"

arm_description = {
    "manufacturer": "ur",
    "model": "05",
    "version": "e",
    "control_rate": 500,
}

arm_location = {
    "parent_link": "base_link",
    "xyz": [1.0, 2.0, 3.0],
    "rpy": [4.0, 5.0, 6.0],
}

prefix = "robot_"
ros_namespace = "/robot/arm"
mode = "live"

configuration = get_complete_configuration(
    arm_name,
    arm_description,
    arm_location,
)

configuration_yaml = generate_configuration_file_str(configuration)

controllers_configuration = generate_controllers_configuration_file_str(
    "path/to/controllers_template.yaml",
    prefix,
    arm_name,
    configuration,
    ros_namespace,
)

urdf_description = generate_urdf_description_str(
    prefix,
    mode,
    arm_name,
    arm_description,
    arm_location,
    ros_namespace,
    additional_urdf_arguments={},
)
```

The returned values can be written to files if needed.

---

## Usage

This package is typically used together with:

* `romea_arm_meta_bringup` → generates launch files and handles bringup

---

## Supported arms

Currently, this package supports **Universal Robots (UR) arms only**.
Support for additional robotic arms will be added in the future.

---
