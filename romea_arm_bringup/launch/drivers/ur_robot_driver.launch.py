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


from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    device = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    rate = LaunchConfiguration("rate").perform(context)

    driver = LaunchDescription()

    driver_node = Node(
        package="romea_ublox_driver",
        executable="ublox_driver_node",
        name="driver",
        output="screen",
        parameters=[
            {"device": device},
            {"baudrate": int(baudrate)},
            {"frame_id": frame_id},
            {"rate": int(rate)},
        ],
    )

    driver.add_action(driver_node)

    return [driver]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("device"))
    declared_arguments.append(DeclareLaunchArgument("baudrate"))
    declared_arguments.append(DeclareLaunchArgument("frame_id"))
    declared_arguments.append(DeclareLaunchArgument("rate"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
