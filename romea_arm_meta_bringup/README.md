# 1) Overview #

The romea_arm_bringup package provides  : 

 - launch files able to launch ros2 arm drivers according a meta-description file provided by user (see next section for arm meta-description file overview), only drivers for Universal Robot arms are supported for now :

   - [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_arm_bringup arm_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - launch files able to launch ros2 arm controller according a meta-description file provided by user, supported controllers are provided from [ros2 controllers](https://github.com/ros-controls/ros2_controllers) package :

   - joint_trajectory_controller
   - scaled_joint_trajectory_controller
   - forward_velocity_controller
   - forward_position_controller

   It is possible to launch one or several controllers via command line : 

    ```console
    ros2 launch romea_arm_bringup arm_controllers.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - a python module able to load and parse arm meta-description file as well as to create URDF description of the arm according a given meta-description.

 - a ros2 python executable able to create arm URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_arm_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > arm.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and ensor URDFs to create a complete URDF description of the robot.  

   



# 2) Arm meta-description #

As seen below arm meta-description file is a yaml file constituted by five items. The first item is the name of the arm defined by user. The second one is the configuration of ROS2 driver used to communicate with the arm (see section 4 for more explanations). The third item is the configuration of ROS2 controllers used to control the arm (see section 4 for more explanations). The fourth item provides basics specifications of the arm and the fifth item specifies where the arm is located on the robot, these informations will be used to create URDF description and by user to configure its algorithms.  Finally, the last item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, arm topics are always the same names for each drivers, controllers and simulator plugins.       

Example :
```yaml
  name: arm # name of the arm given by user
  driver: # arm driver configuration
    pkg: ur_robot_driver  # ros2 driver package choiced by user and its parameters 
    ip: "192.168.1.101"
  control:
    controller_manager: # controller manager configuration
      update_rate: 500
      configuration: # controller manager yaml configuration file path
        pkg: romea_arm_bringup
        file: config/ur_controller_manager.yaml
    controllers: # controllers configuration
      selected: [joint_trajectory_controller] # list of started controllers
      configuration: # controllers yaml configuration file
        pkg: romea_arm_bringup
        file: config/ur_controllers.yaml
  configuration:  # arm basic specifications
    type: ur  # type of arm
    model: 5e  # model of arm
    calibration: # calibration file path
      pkg: ur_description
      file: config/ur5e/default_kinematics.yaml
    joint_limits: # joint limits file path
      pkg: ur_description
      file: config/ur5e/joint_limits.yaml
    initial_joint_positions: #  initial joint positions file path
      pkg: ur_description
      file: config/initial_positions.yaml
  geometry: # geometry configuration 
    parent_link: "base_link" # name of parent link where is located the arm
    xyz: [1.0, 2.0, 3.0]  # position of the arm
    rpy: [4.0, 5.0, 6.0]  # orientation of the arm
  records:
    joint_states: true
```

# 4) Supported arm models

Supported arm receiver are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| universal robot |    ur10     |
| universal robot |    ur5e     |

# 5) Supported arm ROS2 drivers

Only driver from Universal Robot arms are supported for now  [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver). In order to used it, you can specify driver item in arm meta-description file like this:

- UR robot driver:

```yaml
  pkg: "ur_robot_driver"  # ROS2 package name  
    ip:  "192.168.1.101"  # ip of the arm
```

<!-- For each driver a python launch file with the name of the ROS2 package is provided in launch directory. When the meta-description is read by the main launch file called arm_driver.launch.py the corresponding driver is automatically launched taking into account parameters define by user. -->

# 5) Supported arm ROS2 controllers

Only controllers from [ros2_controllers package](https://github.com/ros-controls/ros2_controllers). In order to used one of them, you need to specify control item in arm meta-description file like this:

```yaml
  control:
    controller_manager: # controller manager configuration
      update_rate: 500  # rate of controller manager 
      configuration: # controller manager yaml configuration file path
        pkg: romea_arm_bringup
        file: config/ur_controller_manager.yaml
    controllers: # controllers configuration
      selected: [joint_trajectory_controller] # list of started controllers
      configuration: # controllers yaml configuration file
        pkg: romea_arm_bringup
        file: config/ur_controllers.yaml
```

Where you need to define controller_manager item by setting the control rate frequency and providing the path of the configuration file containing the list of controllers that can be used to control the arm (see example below). 

```yaml
controller_manager:
  ros__parameters:
    joint_state_broadcaster: # name of the controller
      type: joint_state_broadcaster/JointStateBroadcaster # type of the controller

    joint_trajectory_controller: # name of the controller
      type: joint_trajectory_controller/JointTrajectoryController # type of the controller

    forward_velocity_controller: # name of the controller
      type: velocity_controllers/JointGroupVelocityController # type of the controller

    forward_position_controller: # name of the controller
      type: position_controllers/JointGroupPositionController # type of the controller
```
And where you need to define controllers item by providing the path of the file containing their configuration (see example below  for an ur arm  controlled by an joint_trajectory controller).

```yaml
joint_trajectory_controller: #controller name, don't add namespace
  ros__parameters:
    joints: # list of joints to control
      - shoulder_pan_joint  #shouler pan joint name, don't add prefix
      - shoulder_lift_joint #shouler lift joint name, don't add prefix
      - elbow_joint #elbow joint name, don't add prefix
      - wrist_1_joint #wrist_1 joint name, don't add prefix
      - wrist_2_joint #wrist_2 joint name, don't add prefix
      - wrist_3_joint #wrist_3 joint name, don't add prefix
    command_interfaces:  #Command interfaces provided by the hardware interface for all joints.
      - position  
    state_interfaces: # State interfaces provided by the hardware for all joints
      - position
      - velocity
    state_publish_rate: 100.0 # Frequency (in Hz) at which the controller state is published.
    action_monitor_rate: 20.0 # Frequency (in Hz) at which the action goal status is monitored
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint: { trajectory: 0.2, goal: 0.1 } #constraints for shouler pan joint, don't add prefix before joint name
      shoulder_lift_joint: { trajectory: 0.2, goal: 0.1 } #constraints for shouler lift joint, don't add prefix before joint name
      elbow_joint: { trajectory: 0.2, goal: 0.1 } #constraints for elbow joint, don't add prefix before joint name
      wrist_1_joint: { trajectory: 0.2, goal: 0.1 } #constraints for wrist1 joint, don't add prefix before joint name
      wrist_2_joint: { trajectory: 0.2, goal: 0.1 } #constraints for wrist2 joint, don't add prefix before joint name
      wrist_3_joint: { trajectory: 0.2, goal: 0.1 } #constraints for wrist3 joint, don't add prefix before joint name
```

Please note that no namespace for controller names and no prefix for joint names must be specified in the configuration file. They will be added automatically by the launch file script according to the robot and arm names. For example, if we have a robot called robot1 with an arm called arm1, then the controllers will be launched in the namespace /robot1/arm1 and the prefix robot1_arm1_ will be added to all joint names.