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

from ament_index_python.packages import get_package_share_directory

import pytest

from romea_arm_description import (
    get_complete_configuration,
    get_specifications,
    get_specifications_file_path,
)


@pytest.fixture(scope="module")
def user_description():

    return {
        "manufacturer": "ur",
        "model": "05",
        "version": "e",
    }


def test_get_specifications_file_path_ok(user_description):
    assert (
        get_specifications_file_path(user_description)
        == get_package_share_directory("romea_arm_description")
        + "/config/ur_xx_e_specifications.yaml"
    )


def test_get_specifications_ok(user_description):
    assert (
        get_specifications(user_description)["home_joint_positions"]["defaults"][
            "shoulder_lift_joint"
        ]
        == -90.0
    )


def test_get_complete_configuration_ok(user_description):

    configuration = get_complete_configuration("arm", user_description, {})

    assert configuration["model"] == "05"
    assert configuration["version"] == "e"
    assert configuration["control_rate"] == 500
    assert configuration["home_joint_positions"]["shoulder_lift_joint"] == -90.0
