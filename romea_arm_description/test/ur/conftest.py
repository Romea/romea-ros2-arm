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

from romea_arm_description import get_specifications


def supported_arms():
    arms = []
    for version in ("cb", "e"):
        description = {"manufacturer": "ur", "model": "05", "version": version}
        for model in get_specifications(description)["model"]["list"]:
            arms.append(pytest.param((model, version), id=f"ur{model}_{version}"))
    return arms


@pytest.fixture(scope="module", params=supported_arms())
def user_description(request):
    model, version = request.param
    return {"manufacturer": "ur", "model": model, "version": version}


@pytest.fixture(scope="module")
def arm_location():
    return {"parent_link": "world", "xyz": [1.0, 2.0, 3.0], "rpy": [4.0, 5.0, 6.0]}
