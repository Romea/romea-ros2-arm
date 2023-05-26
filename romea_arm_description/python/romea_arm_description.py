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


import xacro
from xml.dom import minidom, Node
from ament_index_python.packages import get_package_share_directory


def ur_arm_urdf(prefix, mode, name, model,
                parent_link, xyz, rpy,
                ip, controller_manager_config_yaml_file,
                ros_prefix):

    xacro_file = (
        get_package_share_directory("romea_arm_description")
        + "/urdf/"
        + "ur_"
        + model
        + ".xacro.urdf"
    )

    ros2_control_config_urdf_file = "/tmp/"+prefix+name+"_ros2_control.urdf"

    arm_urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "name": name,
            "model": model,
            "parent_link": parent_link,
            "xyz": " ".join(map(str, xyz)),
            "rpy": " ".join(map(str, rpy)),
            "ip": ip,
            "controller_manager_config_yaml_file": controller_manager_config_yaml_file,
            "ros2_control_config_urdf_file": ros2_control_config_urdf_file,
            "ros_prefix": ros_prefix
        },
    )

    ros_control_urdf_xml = minidom.Document()
    ros_control_urdf_xml_root = ros_control_urdf_xml.createElement("robot")
    ros_control_urdf_xml_root.setAttribute("name", prefix+name+"_ros2_control")
    ros_control_urdf_xml_root.appendChild(arm_urdf_xml.getElementsByTagName("ros2_control")[0])
    ros_control_urdf_xml.appendChild(ros_control_urdf_xml_root)

    with open(ros2_control_config_urdf_file, "w") as f:
        f.write(ros_control_urdf_xml.toprettyxml())

    return arm_urdf_xml.toprettyxml()
