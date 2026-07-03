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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


class LaunchVariables:
    def __init__(self, context):
        self.__context = context

    def get(self, variable_name):
        return LaunchConfiguration(variable_name).perform(self.__context)


def launch_setup(context, *args, **kwargs):

    var = LaunchVariables(context)
    active_controller = var.get("active_controller")
    go_home_controller = var.get("go_home_controller")
    controller_manager = var.get("controller_manager")
    launch_controller_manager = var.get("launch_controller_manager")

    launch = LaunchDescription()

    if launch_controller_manager == "true":
        controller_manager_parameters = ParameterFile(
            [
                FindPackageShare("romea_arm_meta_bringup"),
                "/config/ur/controller_manager_",
                var.get("mode"),
                ".yaml",
            ],
            allow_substs=True
        )

        controller_manager_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            name=controller_manager,
            parameters=[controller_manager_parameters],
            output="screen",
        )
        launch.add_action(controller_manager_node)

    controller_parameters = ParameterFile(
        var.get("controller_configurations_file_path"),
        allow_substs=True
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[controller_parameters],
        arguments=[
            "joint_state_broadcaster",
            "-c",
            controller_manager
        ],
    )
    launch.add_action(joint_state_broadcaster_spawner)

    active_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[controller_parameters],
        arguments=[
            active_controller,
            "-c",
            controller_manager
        ],
    )
    launch.add_action(active_controller_spawner)

    if active_controller != go_home_controller:

        go_home_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            parameters=[controller_parameters],
            arguments=[
                "--inactive",
                go_home_controller,
                "-c",
                controller_manager
            ],
        )

        launch.add_action(go_home_controller_spawner)

    return [launch]


def generate_launch_description():

    mode = LaunchConfiguration("mode")
    default_trajectory_controller = PythonExpression(
        [
            "'joint_trajectory_controller' if 'simulation' in '",
            mode,
            "' else 'scaled_joint_trajectory_controller'",
        ]
    )
    default_launch_controller_manager = PythonExpression(
        [
            "'false' if '",
            mode,
            "' in ['simulation_gazebo', 'simulation_gazebo_classic'] else 'true'",
        ]
    )
    default_controller_configurations_file_path = [
        FindPackageShare("romea_arm_meta_bringup"),
        "/config/ur/controller_configurations_",
        mode,
        ".yaml",

    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_manager",
                default_value="controller_manager"
            ),
            DeclareLaunchArgument(
                "launch_controller_manager",
                default_value=default_launch_controller_manager,
            ),
            DeclareLaunchArgument(
                "controller_configurations_file_path",
                default_value=default_controller_configurations_file_path
            ),
            DeclareLaunchArgument(
                "active_controller",
                default_value=default_trajectory_controller
            ),
            DeclareLaunchArgument(
                "go_home_controller",
                default_value=default_trajectory_controller
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


# #
# # Author: Denis Stogl

# from launch import LaunchDescription
# from launch.actions import (
#     DeclareLaunchArgument,
#     IncludeLaunchDescription,
#     OpaqueFunction,
#     ExecuteProcess,
# )
# from launch.conditions import IfCondition, UnlessCondition
# from launch.launch_description_sources import AnyLaunchDescriptionSource
# from launch.substitutions import (
#     AndSubstitution,
#     LaunchConfiguration,
#     NotSubstitution,
#     PathJoinSubstitution,
# )
# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterFile
# from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


# def launch_setup(context):


#         ros2_control_description_node = Node(
#         package="romea_common_meta_bringup",
#         executable="urdf_broadcaster_node",
#         name="ros2_control_description",
#         parameters=[
#             {
#                 "robot_description": utils.complete_robot_description(
#                     robot_urdf_description, [robot_ros2_control_description]
#                 )
#             }
#         ],
#     )


#     # Initialize Arguments
#     ur_type = LaunchConfiguration("ur_type")
#     robot_ip = LaunchConfiguration("robot_ip")
#     # General arguments
#     controllers_file = LaunchConfiguration("controllers_file")
#     description_launchfile = LaunchConfiguration("description_launchfile")
#     use_mock_hardware = LaunchConfiguration("use_mock_hardware")
#     controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
#     initial_joint_controller = LaunchConfiguration("initial_joint_controller")
#     activate_joint_controller = LaunchConfiguration("activate_joint_controller")
#     launch_rviz = LaunchConfiguration("launch_rviz")
#     rviz_config_file = LaunchConfiguration("rviz_config_file")
#     headless_mode = LaunchConfiguration("headless_mode")
#     launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
#     use_tool_communication = LaunchConfiguration("use_tool_communication")
#     tool_device_name = LaunchConfiguration("tool_device_name")
#     tool_tcp_port = LaunchConfiguration("tool_tcp_port")

#     control_node = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[
#             LaunchConfiguration("update_rate_config_file"),
#             ParameterFile(controllers_file, allow_substs=True),
#             # We use the tf_prefix as substitution in there, so that's why we keep it as an
#             # argument for this launchfile
#         ],
#         output="screen",
#     )

#     dashboard_client_node = IncludeLaunchDescription(
#         condition=IfCondition(
#             AndSubstitution(launch_dashboard_client, NotSubstitution(use_mock_hardware))
#         ),
#         launch_description_source=AnyLaunchDescriptionSource(
#             PathJoinSubstitution(
#                 [FindPackageShare("ur_robot_driver"), "launch", "ur_dashboard_client.launch.py"]
#             )
#         ),
#         launch_arguments={
#             "robot_ip": robot_ip,
#         }.items(),
#     )

#     robot_state_helper_node = Node(
#         package="ur_robot_driver",
#         executable="robot_state_helper",
#         name="ur_robot_state_helper",
#         output="screen",
#         condition=UnlessCondition(use_mock_hardware),
#         parameters=[
#             {"headless_mode": headless_mode},
#             {"robot_ip": robot_ip},
#         ],
#     )

#     tool_comm_path = PathJoinSubstitution(
#         [
#             FindPackagePrefix("ur_client_library"),
#             "lib",
#             "ur_client_library",
#             "tool_communication.py",
#         ]
#     )

#     tool_communication_script = ExecuteProcess(
#         name="ur_tool_comm",
#         condition=IfCondition(use_tool_communication),
#         cmd=[
#             tool_comm_path,
#             robot_ip,
#             "--tcp-port",
#             tool_tcp_port,
#             "--device-name",
#             tool_device_name,
#         ],
#         output="screen",
#     )

#     urscript_interface = Node(
#         package="ur_robot_driver",
#         executable="urscript_interface",
#         parameters=[{"robot_ip": robot_ip}],
#         output="screen",
#         condition=UnlessCondition(use_mock_hardware),
#     )

#     controller_stopper_node = Node(
#         package="ur_robot_driver",
#         executable="controller_stopper_node",
#         name="controller_stopper",
#         output="screen",
#         emulate_tty=True,
#         condition=UnlessCondition(use_mock_hardware),
#         parameters=[
#             {"headless_mode": headless_mode},
#             {"joint_controller_active": activate_joint_controller},
#             {
#                 "consistent_controllers": [
#                     "io_and_status_controller",
#                     "force_torque_sensor_broadcaster",
#                     "joint_state_broadcaster",
#                     "speed_scaling_state_broadcaster",
#                     "tcp_pose_broadcaster",
#                     "ur_configuration_controller",
#                 ]
#             },
#         ],
#     )

#     rviz_node = Node(
#         package="rviz2",
#         condition=IfCondition(launch_rviz),
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", rviz_config_file],
#     )

#     trajectory_until_node = Node(
#         package="ur_robot_driver",
#         executable="trajectory_until_node",
#         name="trajectory_until_node",
#         output="screen",
#         remappings=[
#             (
#                 "/motion_controller/follow_joint_trajectory",
#                 f"/{initial_joint_controller.perform(context)}/follow_joint_trajectory",
#             ),
#         ],
#     )

#     # Spawn controllers
#     def controller_spawner(controllers, active=True):
#         inactive_flags = ["--inactive"] if not active else []
#         return Node(
#             package="controller_manager",
#             executable="spawner",
#             parameters=[
#                 ParameterFile(controllers_file, allow_substs=True),
#             ],
#             arguments=[
#                 "--controller-manager",
#                 "/controller_manager",
#                 "--controller-manager-timeout",
#                 controller_spawner_timeout,
#             ]
#             + inactive_flags
#             + controllers,
#         )

#     controllers_active = [
#         "joint_state_broadcaster",
#         "io_and_status_controller",
#         "speed_scaling_state_broadcaster",
#         "force_torque_sensor_broadcaster",
#         "tcp_pose_broadcaster",
#         "ur_configuration_controller",
#         "friction_model_controller",
#     ]
#     controllers_inactive = [
#         "scaled_joint_trajectory_controller",
#         "joint_trajectory_controller",
#         "forward_velocity_controller",
#         "forward_position_controller",
#         "forward_effort_controller",
#         "force_mode_controller",
#         "passthrough_trajectory_controller",
#         "freedrive_mode_controller",
#         "tool_contact_controller",
#         "motion_primitive_forward_controller",
#     ]
#     if activate_joint_controller.perform(context) == "true":
#         controllers_active.append(initial_joint_controller.perform(context))
#         controllers_inactive.remove(initial_joint_controller.perform(context))

#     if use_mock_hardware.perform(context) == "true":
#         controllers_active.remove("tcp_pose_broadcaster")

#     controller_spawners = [
#         controller_spawner(controllers_active),
#         controller_spawner(controllers_inactive, active=False),
#     ]

#     rsp = IncludeLaunchDescription(
#         AnyLaunchDescriptionSource(description_launchfile),
#         launch_arguments={
#             "robot_ip": robot_ip,
#             "ur_type": ur_type,
#         }.items(),
#     )

#     nodes_to_start = [
#         control_node,
#         dashboard_client_node,
#         robot_state_helper_node,
#         tool_communication_script,
#         controller_stopper_node,
#         urscript_interface,
#         rsp,
#         rviz_node,
#         trajectory_until_node,
#     ] + controller_spawners

#     return nodes_to_start


# def generate_launch_description():
#     declared_arguments = []
#     # UR specific arguments
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "ur_type",
#             description="Type/series of used UR robot.",
#             choices=[
#                 "ur3",
#                 "ur5",
#                 "ur10",
#                 "ur3e",
#                 "ur5e",
#                 "ur7e",
#                 "ur10e",
#                 "ur12e",
#                 "ur16e",
#                 "ur8long",
#                 "ur15",
#                 "ur18",
#                 "ur20",
#                 "ur30",
#             ],
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "robot_ip", description="IP address by which the robot can be reached."
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "safety_limits",
#             default_value="true",
#             description="Enables the safety limits controller if true.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "safety_pos_margin",
#             default_value="0.15",
#             description="The margin to lower and upper limits in the safety controller.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "safety_k_position",
#             default_value="20",
#             description="k-position factor in the safety controller.",
#         )
#     )
#     # General arguments
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "controllers_file",
#             default_value=PathJoinSubstitution(
#                 [FindPackageShare("ur_robot_driver"), "config", "ur_controllers.yaml"]
#             ),
#             description="YAML file with the controllers configuration.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "description_launchfile",
#             default_value=PathJoinSubstitution(
#                 [FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"]
#             ),
#             description="Launchfile (absolute path) providing the description. "
#             "The launchfile has to start a robot_state_publisher node that "
#             "publishes the description topic.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tf_prefix",
#             default_value="",
#             description="tf_prefix of the joint names, useful for "
#             "multi-robot setup. If changed, also joint names in the controllers' configuration "
#             "have to be updated.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "use_mock_hardware",
#             default_value="false",
#             description="Start robot with mock hardware mirroring command to its states.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "mock_sensor_commands",
#             default_value="false",
#             description="Enable mock command interfaces for sensors
# used for simple simulations."
#             "Used only if 'use_mock_hardware' parameter is true.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "headless_mode",
#             default_value="false",
#             description="Enable headless mode for robot control",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "controller_spawner_timeout",
#             default_value="10",
#             description="Timeout used when spawning controllers.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "initial_joint_controller",
#             default_value="scaled_joint_trajectory_controller",
#             choices=[
#                 "scaled_joint_trajectory_controller",
#                 "joint_trajectory_controller",
#                 "forward_velocity_controller",
#                 "forward_position_controller",
#                 "freedrive_mode_controller",
#                 "passthrough_trajectory_controller",
#                 "motion_primitive_forward_controller",
#             ],
#             description="Initially loaded robot controller.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "activate_joint_controller",
#             default_value="true",
#             description="Activate loaded joint controller.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "rviz_config_file",
#             default_value=PathJoinSubstitution(
#                 [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
#             ),
#             description="RViz config file (absolute path) to use when launching rviz.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "launch_dashboard_client",
#             default_value="true",
#             description="Launch Dashboard Client?",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "use_tool_communication",
#             default_value="false",
#             description="Only available for e series!",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_parity",
#             default_value="0",
#             description="Parity configuration for serial communication. Only effective, if "
#             "use_tool_communication is set to True.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_baud_rate",
#             default_value="115200",
#             description="Baud rate configuration for serial communication. Only effective, if "
#             "use_tool_communication is set to True.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_stop_bits",
#             default_value="1",
#             description="Stop bits configuration for serial communication. Only effective, if "
#             "use_tool_communication is set to True.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_rx_idle_chars",
#             default_value="1.5",
#             description="RX idle chars configuration for serial communication. Only effective, "
#             "if use_tool_communication is set to True.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_tx_idle_chars",
#             default_value="3.5",
#             description="TX idle chars configuration for serial communication. Only effective, "
#             "if use_tool_communication is set to True.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_device_name",
#             default_value="/tmp/ttyUR",
#             description="File descriptor that will be generated for the
# tool communication device. "
#             "The user has be be allowed to write to this location. "
#             "Only effective, if use_tool_communication is set to True.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_tcp_port",
#             default_value="54321",
#             description="Remote port that will be used for bridging the tool's serial device. "
#             "Only effective, if use_tool_communication is set to True.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "tool_voltage",
#             default_value="0",  # 0 being a conservative value that won't destroy anything
#             description="Tool voltage that will be setup.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "reverse_ip",
#             default_value="0.0.0.0",
#             description="IP that will be used for the robot controller to
# communicate back to the driver.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "script_command_port",
#             default_value="50004",
#             description="Port that will be opened to forward URScript commands to the robot.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "reverse_port",
#             default_value="50001",
#             description="Port that will be opened to send cyclic instructions
# from the driver to the robot controller.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "script_sender_port",
#             default_value="50002",
#             description="The driver will offer an interface to query
# the external_control URScript on this port.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "trajectory_port",
#             default_value="50003",
#             description="Port that will be opened for trajectory control.",
#         )
#     )
#     declared_arguments.append(
#         DeclareLaunchArgument(
#             name="update_rate_config_file",
#             default_value=[
#                 PathJoinSubstitution(
#                     [
#                         FindPackageShare("ur_robot_driver"),
#                         "config",
#                     ]
#                 ),
#                 "/",
#                 LaunchConfiguration("ur_type"),
#                 "_update_rate.yaml",
#             ],
#         )
#     )
#     return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
