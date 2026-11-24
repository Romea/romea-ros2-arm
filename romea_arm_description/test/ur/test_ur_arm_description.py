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

from romea_arm_description import (
    get_complete_configuration,
    get_specifications,
    get_specifications_file_path,
)


HOME_JOINT_POSITIONS = {
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -90.0,
    "elbow_joint": 0.0,
    "wrist_1_joint": -90.0,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
}


def test_get_specifications_file_path_ok(user_description):
    version = user_description["version"]
    assert get_specifications_file_path(user_description) == (
        get_package_share_directory("romea_arm_description")
        + f"/config/ur_xx_{version}_specifications.yaml"
    )


def test_get_specifications_ok(user_description):
    specifications = get_specifications(user_description)
    assert user_description["model"] in specifications["model"]["list"]
    assert specifications["control_rate"] == (125 if user_description["version"] == "cb" else 500)
    assert specifications["home_joint_positions"]["defaults"] == HOME_JOINT_POSITIONS


def test_get_complete_configuration_ok(user_description, arm_location):
    configuration = get_complete_configuration("arm", user_description, arm_location)
    assert configuration["manufacturer"] == "ur"
    assert configuration["model"] == user_description["model"]
    assert configuration["version"] == user_description["version"]
    assert configuration["control_rate"] == (125 if user_description["version"] == "cb" else 500)
    assert configuration["home_joint_positions"] == HOME_JOINT_POSITIONS
    assert configuration["parent_link"] == "world"
    assert configuration["xyz"] == [1.0, 2.0, 3.0]
    assert configuration["rpy"] == [4.0, 5.0, 6.0]
