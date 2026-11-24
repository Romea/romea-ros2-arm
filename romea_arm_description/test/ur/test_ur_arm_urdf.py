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

import math
import os
import xml.etree.ElementTree as ET

import pytest
from romea_arm_description import generate_urdf_description_str


JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
HOME_POSITIONS = [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]


@pytest.fixture(scope="module", params=["live", "simulation"])
def urdf_xml(request, user_description, arm_location):
    distro = os.environ["ROS_DISTRO"]
    mode = request.param
    if mode == "simulation":
        mode = "simulation_gazebo_classic" if distro == "humble" else "simulation_gazebo"
    description = generate_urdf_description_str(
        "robot_", mode, "arm", user_description, arm_location, "/robot/ns/arm",
        {
            "ros_distro": distro,
            "standalone": "true",
            "generate_gazebo_tag": "true" if request.param == "simulation" else "false",
            "generate_ros2_control_tag": "true",
            "controllers_config_yaml_file": os.path.join(
                os.path.dirname(__file__), "test_ur_arm_ros2_controller_manager.yaml"
            ),
        },
    )
    return ET.fromstring(description)


def test_arm_links_and_joints(urdf_xml):
    for name in ("base_link", "shoulder_link", "upper_arm_link", "forearm_link",
                 "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"):
        assert urdf_xml.find(f"link[@name='robot_arm_{name}']") is not None
    joints = [joint for joint in urdf_xml.findall("joint")
              if joint.get("type") in ("revolute", "continuous")]
    assert {joint.get("name") for joint in joints} == {
        "robot_arm_" + name for name in JOINT_NAMES
    }


def test_arm_mounting_pose(urdf_xml):
    assert urdf_xml.find("link[@name='world']") is not None
    joint = urdf_xml.find("joint[@name='robot_arm_base_joint']")
    assert joint.get("type") == "fixed"
    assert joint.find("parent").get("link") == "world"
    assert joint.find("child").get("link") == "robot_arm_base_link"
    origin = joint.find("origin")
    assert list(map(float, origin.get("xyz").split())) == [1.0, 2.0, 3.0]
    assert list(map(float, origin.get("rpy").split())) == pytest.approx(
        [math.radians(value) for value in [4.0, 5.0, 6.0]]
    )


def test_arm_initial_joint_positions(urdf_xml):
    control = urdf_xml.find("ros2_control")
    assert control is not None
    for name, expected in zip(JOINT_NAMES, HOME_POSITIONS):
        joint = control.find(f"joint[@name='robot_arm_{name}']")
        assert joint is not None
        initial = joint.findtext("state_interface[@name='position']/param[@name='initial_value']")
        assert float(initial) == pytest.approx(expected)


def test_arm_simulation_plugin(urdf_xml):
    plugin = urdf_xml.find("gazebo/plugin")
    if urdf_xml.findtext("ros2_control/hardware/plugin") in (
        "gazebo_ros2_control/GazeboSystem", "gz_ros2_control/GazeboSimSystem"
    ):
        assert plugin is not None
        assert plugin.findtext("ros/namespace") == "/robot/ns/arm"
        expected = "libgazebo_ros2_control.so" if os.environ["ROS_DISTRO"] == "humble" else (
            "gz_ros2_control-system"
        )
        assert plugin.get("filename") == expected
    else:
        assert plugin is None
