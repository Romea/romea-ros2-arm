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

from romea_arm_bringup import ArmMetaDescription


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_arm_bringup.yaml")
    return ArmMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "arm"


# def test_get_namespace(meta_description):
#     assert meta_description.get_namespace() == "ns"


def test_has_driver_configuration(meta_description):
    assert meta_description.has_driver_configuration() is True


def test_get_driver_pkg(meta_description):
    assert meta_description.get_driver_pkg() == "ur_robot_driver"


def test_get_driver_ip(meta_description):
    assert meta_description.get_driver_ip() == "192.168.1.101"


def test_get_type(meta_description):
    assert meta_description.get_type() == "ur"


def test_get_model(meta_description):
    assert meta_description.get_model() == "5e"


def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]


def test_get_rpy(meta_description):
    assert meta_description.get_rpy() == [4.0, 5.0, 6.0]
