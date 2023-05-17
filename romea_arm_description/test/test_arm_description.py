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


import pytest
import xml.etree.ElementTree as ET
from romea_arm_description import ur_arm_urdf


@pytest.fixture(scope="module")
def ur_urdf_xml():
    prefix = "robot_"
    mode = "simulation"
    name = "arm"
    model = "5e"
    parent_link = "base_link"
    xyz = [1.0, 2.0, 3.0]
    rpy = [4.0, 5.0, 6.0]
    ip = "192.168.1.101"
    controllers_yaml_file = "controller.yaml"
    ros_prefix = "/ns/"

    return ET.fromstring(ur_arm_urdf(prefix, mode, name, model,
                                     parent_link, xyz, rpy,
                                     ip, controllers_yaml_file,
                                     ros_prefix))


def test_ur_name(ur_urdf_xml):
    assert ur_urdf_xml.find("link").get("name") == "robot_arm_base_link"


def test_ur_position(ur_urdf_xml):
    assert ur_urdf_xml.find("joint/origin").get("xyz") == " 1.0 2.0 3.0"


def test_ur_orientation(ur_urdf_xml):
    assert ur_urdf_xml.find("joint/origin").get("rpy") == " 4.0 5.0 6.0"


def test_ur_parent_link(ur_urdf_xml):
    assert ur_urdf_xml.find("joint/parent").get("link") == "robot_base_link"


# def test_plugin_namespace(ur_urdf_xml):
#     assert ur_urdf_xml.find("gazebo/sensor/plugin/ros/namespace").text == "ns"
