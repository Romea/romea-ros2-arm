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

from romea_arm_meta_bringup.meta_description import (
    ArmMetaDescription,
    get_complete_configuration,
    get_specifications,
)


@pytest.fixture(scope="module")
def meta_description():
    meta_description_file_path = os.path.join(os.getcwd(), "test_arm_meta_bringup.yaml")
    return ArmMetaDescription(meta_description_file_path)


def test_get_name(meta_description):
    assert meta_description.get_name() == "arm"


def test_get_namespace(meta_description):
    assert meta_description.get_namespace() == "ns"


def test_get_manufacturer(meta_description):
    assert meta_description.get_manufacturer() == "ur"


def test_get_model(meta_description):
    assert meta_description.get_model() == "05"


def test_get_version(meta_description):
    assert meta_description.get_version() == "e"


def test_get_launch_file(meta_description):
    assert meta_description.get_launch_file() is not None


def test_get_parent_link(meta_description):
    assert meta_description.get_parent_link() == "base_link"


def test_get_xyz(meta_description):
    assert meta_description.get_xyz() == [1.0, 2.0, 3.0]


def test_get_rpy(meta_description):
    assert meta_description.get_rpy() == [4.0, 5.0, 6.0]


def test_get_records(meta_description):
    records = meta_description.get_records()
    assert records["joint_states"] is False


def test_get_specifications(meta_description):
    arm_specifactions = get_specifications(meta_description)
    assert arm_specifactions['control_rate'] == 500


def test_get_complete_configuration(meta_description):
    arm_configuration = get_complete_configuration(meta_description)
    assert arm_configuration['control_rate'] == 500
