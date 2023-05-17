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


import os
import pytest
import subprocess

from ament_index_python import get_package_prefix

import xml.etree.ElementTree as ET


@pytest.fixture(scope="module")
def urdf():

    exe = (
        get_package_prefix("romea_arm_bringup")
        + "/lib/romea_arm_bringup/urdf_description.py"
    )

    meta_description_file_path = os.path.join(os.getcwd(), "test_arm_bringup.yaml")

    return ET.fromstring(
        subprocess.check_output(
            [
                exe,
                "robot_namespace:robot",
                "mode:simulation",
                "meta_description_file_path:" + meta_description_file_path,
            ],
            encoding="utf-8",
        )
    )


def test_arm_name(urdf):
    assert urdf.find("link").get("name") == "robot_arm_base_link"


def test_arm_position(urdf):
    assert urdf.find("joint/origin").get("xyz") == " 1.0 2.0 3.0"


def test_arm_orientation(urdf):
    assert urdf.find("joint/origin").get("rpy") == " 4.0 5.0 6.0"


def test_arm_parent_link(urdf):
    assert urdf.find("joint/parent").get("link") == "robot_base_link"


# def test_plugin_namespace(urdf):
#     assert urdf.find("gazebo/sensor/plugin/ros/namespace").text == "/robot/ns/gps"
