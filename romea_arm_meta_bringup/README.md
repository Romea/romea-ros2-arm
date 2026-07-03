# romea_arm_meta_bringup

## Overview

romea_arm_meta_bringup provides tools to describe and launch robotic arms using a meta-description approach.

It allows defining arm systems in a high-level YAML format and automatically generating consistent ROS 2 artifacts such as:

* configuration files → used as generic ROS 2 configuration inputs
* controllers configuration files → used by the ROS 2 controller manager
* launch files → used to start arm drivers and controllers
* URDF description files → used to load the arm into simulators

This package is built on top of `romea_common_meta_bringup` and specializes it for robotic arm integration. 

It also provides launch files that allow controlling the arm both on a real robot and in simulation.

---

## Arm meta-description concept

An `arm meta-description` is a YAML file that defines a robotic arm and how it should be integrated into a system. It centralizes:

* arm identification (name, namespace)
* hardware + control configuration (manufacturer, model, version)
* kinematic attachment (parent link, pose)
* ROS2 launch description

---

### Example meta-description

```yaml id="9lfc1h"
name: arm
namespace: ns

configuration:
  manufacturer: ur
  model: "05"
  version: e
  control_rate: 500
  home_joint_positions:
    shoulder_pan_joint: 0.0
    shoulder_lift_joint: -90.0
    elbow_joint: 0.0
    wrist_1_joint: -90.0
    wrist_2_joint: 0.0
    wrist_3_joint: 0.0

location:
  parent_link: base_link
  xyz: [1.0, 2.0, 3.0]
  rpy: [4.0, 5.0, 6.0]

launch:
  - include:
      file: "$(find-pkg-share romea_arm_meta_bringup)/profile/urdf_broadcaster.py"
  - include:
      file: "$(find-pkg-share romea_arm_meta_bringup)/profile/ur.launch.py"
      arg:
        - name: controller_configurations_file_path
          value: $(var controller_configurations_file_path)
```

### Launch files Profiles


The `profile/` directory contains reusable ROS2 launch files dedicated to arm bringup.

These launch profiles provide predefined setups for common execution contexts, such as:

* broadcasting the generated ros2_control URDF description, for example `urdf_broadcaster.py`
* starting a real UR driver and controller spawners, for example `ur.launch.py`
* reusing standardized bringup configurations across live and simulation modes

Each profile is intended to be included from the launch section of an arm meta-description. This makes it possible to select the appropriate runtime behavior depending on the selected mode, while keeping the meta-description concise and consistent.

---

## Scripts

`romea_arm_meta_bringup` provides several scripts to generate ROS2 artifacts (configuration, controllers configuration, launch and URDF files) from an arm meta-description; the usage and resulting outputs are described below.

### Generate configuration file

```bash id="rq0ybi"
generate-arm-configuration-file \
  meta_description_file_path:=path/to/arm_meta_description.yaml \
  extended:=false
```

#### Example output

```yaml id="pgd8yu"
model: 05
version: e
manufacturer: ur
control_rate: 500  # unit Hz
home_joint_positions:
    shoulder_pan_joint: 0.0  # unit °
    shoulder_lift_joint: -90.0  # unit °
    elbow_joint: 0.0  # unit °
    wrist_1_joint: -90.0  # unit °
    wrist_2_joint: 0.0  # unit °
    wrist_3_joint: 0.0  # unit °
parent_link: base_link
xyz: [1.0, 2.0, 3.0]  # unit m
rpy: [4.0, 5.0, 6.0]  # unit °
```

---

### Generate controller_manager configuration file

Controller manager configuration templates are provided in the `config/` directory and contain placeholders:

* `ros_namespace`
* `control_rate`

These values are automatically replaced:

```bash id="9xg4hf"
generate-arm-controller-manager-configuration-file \
  mode:=live \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/arm_meta_description.yaml
```

#### Example (simplified)

```yaml id="lgp6xe"
/robot/ns/arm/controller_manager:
  ros__parameters:
    update_rate: 500

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
```

---

### Generate controller configurations file

Controller configuration templates are provided in the `config/` directory and contain placeholders:

* `ros_namespace`
* `tf_prefix`
* `control_rate`

These values are automatically replaced:

```bash
generate-arm-controller-configuration-file \
  mode:=live \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/arm_meta_description.yaml
```

#### Example (simplified)

```yaml
/robot/ns/arm/joint_trajectory_controller:
  ros__parameters:
    joints:
      - robot_arm_shoulder_pan_joint
      - robot_arm_shoulder_lift_joint
      - robot_arm_elbow_joint
      - robot_arm_wrist_1_joint
      - robot_arm_wrist_2_joint
      - robot_arm_wrist_3_joint
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

---

### Generate URDF description

Generates the arm URDF description from the meta-description.

```bash
generate-arm-urdf-description \
  mode:=simulation_gazebo \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/arm_meta_description.yaml \
  generate_ros2_control_tag:=true \
  generate_gazebo_tag:=true
```

The generated URDF attaches the arm to the parent link defined in the `location` section. It also applies the robot prefix to generated links and joints and can optionally include ros2_control and Gazebo tags.

#### Example output (simplified)

```xml
<link name="robot_arm_base_link">
  ...
</link>

<joint name="robot_arm_joint" type="fixed">
  <origin xyz="1.0 2.0 3.0"
          rpy="0.06981317007977318 0.08726646259971647 0.10471975511965977"/>
  <parent link="robot_base_link"/>
  <child link="robot_arm_base_link"/>
</joint>

<ros2_control name="robot_arm" type="system">
  <hardware>
    <plugin>...</plugin>
  </hardware>
  ...
</ros2_control>

<gazebo>
  ...
</gazebo>
```

This URDF description can be concatenated with the mobile base and other device URDF descriptions to build a complete robot model.

---

### Generate launch file

Generates a YAML ROS 2 launch file from the meta-description.

```bash
generate-arm-launch-file \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/arm_meta_description.yaml
```

#### Example output

```yaml
launch:
- arg:
    name: mode
    default: live
- arg:
    name: ros2_control_description
    default: $(command 'generate-arm-urdf-description mode:=$(var mode) robot_namespace:=robot meta_description_file_path:=path/to/arm_meta_description.yaml standalone:=true generate_ros2_control_tag:=true' ignore)
- arg:
    name: controller_configurations_file_path
    default: $(find-pkg-share romea_arm_meta_bringup)/config/ur/controller_configurations_$(var mode).yaml
- group:
  - push-ros-namespace: {namespace: robot}
  - push-ros-namespace: {namespace: ns}
  - push-ros-namespace: {namespace: arm}
  - let: {name: model, value: '05'}
  - let: {name: version, value: e}
  - let: {name: manufacturer, value: ur}
  - let: {name: control_rate, value: '500'}
  - let: {name: parent_link, value: base_link}
  - let: {name: xyz, value: '[1.0, 2.0, 3.0]'}
  - let: {name: rpy, value: '[4.0, 5.0, 6.0]'}
  - let: {name: tf_prefix, value: robot_}
  - let: {name: frame_id, value: robot_arm_link}
  - include:
      file: $(find-pkg-share romea_arm_meta_bringup)/profile/urdf_broadcaster.py
  - include:
      file: $(find-pkg-share romea_arm_meta_bringup)/profile/ur.launch.py
      arg:
      - name: controller_configurations_file_path
        value: $(var controller_configurations_file_path)
```

#### Notes

* the launch file is generated from the `launch` section of the meta-description
* namespaces are automatically constructed (`robot → device → arm`)
* all configuration values are exposed as `let` variables
* the selected profile is included at the end
* the ros2_control description is generated by default with `generate-arm-urdf-description`
* the controller configurations file is selected based on the `mode`

This file can be used directly with ROS2 or generated dynamically using `arm.launch.py`.

## Usage

The package provides **two main launch files**:

* `arm.launch.py` → for dynamic bringup (live or simulation mode)
* `simulation_test.launch.py` → for full simulation test

---

### Dynamic bringup

When using `arm.launch.py`, the following steps are performed automatically:

```bash
ros2 launch romea_arm_meta_bringup arm.launch.py \
  mode:=live \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/arm_meta_description.yaml
```

* generation of the arm configuration file
* generation of the launch file
* execution of the generated launch file
* generation and broadcast of the ros2_control description through the generated launch file


#### Live or simulation mode

The `mode` parameter controls the behavior:

* `live` → starts the arm driver and controllers
* `simulation_<simulator>` → starts controllers and simulation bridges

---

### Simulation test

For a complete simulation setup, use:

```bash
ros2 launch romea_arm_meta_bringup simulation_test.launch.py \
  simulator_type:=gazebo \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/arm_meta_description.yaml
```

This launch file:

* starts the simulator
* generates and loads the URDF
* spawns the arm in simulation
* calls `arm.launch.py` to start controllers and bridges

---
## Supported arms

Currently, this package supports **Universal Robots (UR)** arms only. Support for additional robotic arms will be added in the future.
