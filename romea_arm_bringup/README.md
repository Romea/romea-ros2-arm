# 1) Overview #

The romea_gps_bringup package provides  : 

 - launch files able to launch ros2 receiver drivers according a meta-description file provided by user (see next section for GPS meta-description file overview), supported drivers are :

   - [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver)
   - [romea_ublox_driver](https://gitlab.irstea.fr/romea_ros2/interfaces/sensors/romea_ublox)

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_gps_bringup gps_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - a python module able to load and parse GPS meta-description file as well as to create URDF description of the GPS Receiver according a given meta-description.

 - a ros2 python executable able to create GPS URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_gps_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > gps.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and other sensor URDFs to create a complete URDF description of the robot.  

   



# 2) GPS meta-description #

As seen below GPS meta-description file is a yaml file constituted by six items. The first item is the name of sensor defined by user. The second one is the configuration of ROS2 driver used to control GPS receiver (see section 4 for more explanations). The third item is the configuration of ROS2 driver used to deal with NTRIP communication in order to broadcast differential corrections to GPS receiver (see section 5 for more explanations). The fourth item provides basics specifications of the GPS receiver and the fifth item specifies where the GPS receiver antenna is located on the robot, these informations will be used to create URDF description and by user to configure its algorithms.  Finally, the last item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, GPS topics are always the same names for each drivers or simulator plugins.       

Example :
```yaml
  name: "gps"  # name of the gps given by user
  driver: # gps driver configuration
    pkg: "romea_ublox_driver"  # ros2 driver package choiced by user and its parameters 
    device:  "/dev/ttyACM0"
    baudrate: 115200
  ntrip:  # ntrip driver configuration (optional)
    pkg: "ntrip_client"  # ros2 driver package choiced by user and its parameters 
    host: caster.centipede.fr
    port: 2101
    username: centipede # optional
    password: centipede # optional
    mountpoint: MAGC
 configuration: # GPS basic specifications
    type: drotek  #  type of GPS receiver
    model: f9p  # model of GPS receiver
    rate: 10 # frame rate in hz
geometry: # geometry configuration 
  parent_link: "base_link"  # name of parent link where is located the GPS antenna
  xyz: [0.0, 0.0, 1.5]  #and it position in meters
records: # topic to be recorded
  nmea: true # nmea sentences will be recorded into bag
  gps_fix: false # gps_fix topic will not be recorded into bag
  vel: false # vel topic will not be recorded into bag
```

# 4) Supported GPS receiver models

Supported GPS receiver are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| drotek |    f9p     |
| astech | proflex800 |
| ublox  |   evk_m8   |

You can find specifications of each receiver in config directory of romea_gps_description package.

# 5) Supported GPS receiver ROS2 drivers

Supported drivers are [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver) and  [romea_ublox_driver](https://gitlab.irstea.fr/romea_ros2/interfaces/sensors/romea_ublox). In order to used one of them, you can specify driver item in GPS meta-description file like this:

- Nmea Navsat driver:

```yaml
  pkg: "nmea_navsat_driver"  # ROS2 package name  
    device:  "/dev/ttyUSB0"  # serial device
    baudrate: 115200 # serial baudrate
```

* Romea ublox driver:

```yaml
  pkg: "romea_ublox_driver"  # ROS2  package name  
    device:  "/dev/ttyACM0"  # serial device
    baudrate: 115200 # serial baudrate
```

For each driver a python launch file with the name of the ROS2 package is provided in launch directory. When the meta-description is red by the main launch file called gps_driver.launch.py the corresponding driver is automatically launched taking into account parameters define by user. Thanks to remapping defined inside each driver launch files, the data provided by drivers are always published in the same topics called:

- nmea(nmea_msgs/sentence)
- gps_fix(sensor_msgs/NavSatFix)
- vel(geometry_msgd/Twist)  

# 4) Supported NTRIP client ROS2 drivers

Only ROS [ntrip_client](https://github.com/LORD-MicroStrain/ntrip_client) driver is supported for the moment. In order to used it, you can specify ntrip item like this:  

```yaml
  pkg: "ntrip_client"  # ROS package name  
    host: caster.centipede.fr : 
    port: 2101
    username: centipede # optional
    password: centipede # optional
    mountpoint: MAGC : 
```
