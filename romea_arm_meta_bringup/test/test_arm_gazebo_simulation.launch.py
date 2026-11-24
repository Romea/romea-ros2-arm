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


from collections import deque
import math
import os
import time
import unittest
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
import romea_arm_meta_bringup.meta_description as meta_description
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    "robot_arm_shoulder_pan_joint",
    "robot_arm_shoulder_lift_joint",
    "robot_arm_elbow_joint",
    "robot_arm_wrist_1_joint",
    "robot_arm_wrist_2_joint",
    "robot_arm_wrist_3_joint",
]
ARM_NAMESPACE = "/robot/ns/arm"


def get_simulator_type():
    try:
        get_package_share_directory("gazebo_ros")
        return "gazebo_classic"
    except PackageNotFoundError:
        return "gazebo"


def get_meta_description_path():
    return os.path.join(os.path.dirname(__file__), "test_simulation.yaml")


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_arm_meta_bringup")
            + "/launch/simulation_test.launch.py"
        ),
        launch_arguments={
            "gui": "false",
            "simulator_type": get_simulator_type(),
            "robot_namespace": "robot",
            "meta_description_file_path": get_meta_description_path(),
        }.items(),
    )
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestArmGazeboSimulation(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_arm_gazebo_simulation")
        cls.messages = deque(maxlen=5)

        def receive(message):
            cls.messages.append(message)

        cls.subscription = cls.node.create_subscription(
            JointState, ARM_NAMESPACE + "/joint_states", receive, qos_profile_sensor_data
        )
        cls.controllers = cls.node.create_client(
            ListControllers, ARM_NAMESPACE + "/controller_manager/list_controllers"
        )
        cls.trajectory = ActionClient(
            cls.node, FollowJointTrajectory,
            ARM_NAMESPACE + "/joint_trajectory_controller/follow_joint_trajectory",
        )

    @classmethod
    def tearDownClass(cls):
        cls.trajectory.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail("Timed out: " + description)

    def test_arm_world_attachment(self):
        description = meta_description.ArmMetaDescription(get_meta_description_path(), "robot")
        urdf = ET.fromstring(meta_description.generate_xml_urdf_description_str(
            "simulation_" + get_simulator_type(), description,
            {"standalone": "true", "generate_ros2_control_tag": "true"},
        ))
        self.assertIsNotNone(urdf.find("link[@name='world']"))
        joint = urdf.find("joint[@name='robot_arm_base_joint']")
        self.assertIsNotNone(joint)
        self.assertEqual(joint.get("type"), "fixed")
        self.assertEqual(joint.find("parent").get("link"), "world")
        self.assertEqual(joint.find("child").get("link"), "robot_arm_base_link")
        origin = joint.find("origin")
        self.assertEqual(list(map(float, origin.get("xyz").split())), [1.0, 2.0, 3.0])
        for actual, degrees in zip(map(float, origin.get("rpy").split()), [4., 5., 6.]):
            self.assertAlmostEqual(actual, math.radians(degrees))
        self.assertIsNotNone(urdf.find("ros2_control"))

    def test_arm_publishes_joint_states(self):
        self.wait_until(lambda: len(self.messages) == 5, 60, "no arm joint states received")
        stamps = []
        for message in self.messages:
            self.assertEqual(set(message.name), set(JOINT_NAMES))
            self.assertEqual(len(message.position), 6)
            self.assertTrue(all(math.isfinite(value) for value in message.position))
            stamps.append(message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)
        self.assertGreater(stamps[0], 0)
        self.assertTrue(all(b > a for a, b in zip(stamps, stamps[1:])))

    def test_arm_controllers_active(self):
        self.assertTrue(self.controllers.wait_for_service(timeout_sec=60))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            future = self.controllers.call_async(ListControllers.Request())
            self.wait_until(future.done, 10, "controller list not received")
            self.assertIsNone(future.exception())
            controllers = {item.name: item.state for item in future.result().controller}
            if all(controllers.get(name) == "active" for name in (
                "joint_state_broadcaster", "joint_trajectory_controller"
            )):
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail(f"Controllers did not become active: {controllers}")

    def test_arm_follows_trajectory(self):
        self.wait_until(lambda: len(self.messages) == 5, 60, "no arm joint states received")
        self.assertTrue(self.trajectory.wait_for_server(timeout_sec=30))
        positions = dict(zip(self.messages[-1].name, self.messages[-1].position))
        target = [positions[name] for name in JOINT_NAMES]
        target[0] += 0.1
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=target, time_from_start=Duration(sec=2)
        )]
        goal.goal_time_tolerance = Duration(sec=5)
        sent = self.trajectory.send_goal_async(goal)
        self.wait_until(sent.done, 10, "trajectory goal not acknowledged")
        self.assertIsNone(sent.exception())
        handle = sent.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.wait_until(result.done, 20, "trajectory did not complete")
        self.assertIsNone(result.exception())
        self.assertEqual(
            result.result().result.error_code, FollowJointTrajectory.Result.SUCCESSFUL,
            result.result().result.error_string,
        )

        def reached_target():
            actual = dict(zip(self.messages[-1].name, self.messages[-1].position))
            return all(abs(actual[name] - value) < 0.03
                       for name, value in zip(JOINT_NAMES, target))

        self.wait_until(reached_target, 10, "joint positions did not reach the trajectory target")
