# NTRIP Client ROS 

Robeff Technology NTRIP Client package publish RTCM ROS messages and send through serial port.

## Installation 


### Dependencies

* [Robot Operating System (ROS-ROS2)] (Humble)

Installation of ROS2 Humble-> https://docs.ros.org/en/humble/Installation.html

* mavros_msgs
```bash
sudo apt install ros-<distro>-mavros-msgs
```
* nmea_msgs
```bash
sudo apt install ros-<distro>-nmea-msgs
```

### Install Repo

```bash
cd ~/catkin_ws/src
git clone -b RobiOne https://github.com/Robeff-Technology/ntrip_client.git
cd ..
```

### Build

```bash
colcon build --packages-select ntrip_client_ros
```

## Parameters

To connect RTK provider you need to set parameters in config file. You can find config file in `ntrip_client_ros/config/ntrip_client_ros.yaml`. Configure RTK credentials, port and ROS parameters to use ntrip_client package.  


<font size="4">* `NTRIP Parameters`</font>

NTRIP parameters are credentials provided by your RTK provider. Write the IP, Username, password, mount point, port, information provided by your provider in the relevant places in the yaml file.

Parameter | Type    | Detailed Description                    | Default                    |
--- |---------|-----------------------------------------|----------------------------|
ntrip_ip | String  | NTRIP Connection IP                     | "111.111.11.11"            |
ntrip_user_name | String  | NTRIP Username                          | "ABCDEF1234"               |
ntrip_password | String  | NTRIP Password provided by RTK provider | "asdfghj"                  |
ntrip_mount_point | String  | NTRIP Mount point                       | "VRSRTCM31"                |
ntrip_port | Integer | NTRIP connection port                   | 2101                       |
ntrip_location_lat | Double  | Latitude of RTK initial location        | 41.018893949               |
ntrip_location_lon | Double  | Longitude of RTK initial location       | 28.890924848               |


<font size="4">* `ROS Parameters`</font>


The ROS package can both publish the RTCM message as a ROS message from a topic and send it serially from a desired USB port. The package can be used as desired by changing `publish_port_rtcm` or `publish_ros_rtcm_active` parameters.

Parameter | Type    | Detailed Description                                          | Default                    |
--- |---------|---------------------------------------------------------------|----------------------------|
publish_port_rtcm_active | Boolean | Set "true" to send RTCM message through serial USB port       | false                      |
serial_port | String  | Serial port device name of RTK connection                     | "/dev/ttyUSB0"             |
serial_port_baud_rate | Integer | Serial communication baud rate                                | 460800                     |
publish_ros_rtcm_active | Boolean | Publish RTK message as ROS2 topic in mavros_msgs type         | true                       |
rtcm_topic | String  | Topic name that provides all rtcm message in mavros_msgs type | "/sensing/gnss/ntrip/rtcm" |
debug | Boolean | Set 'true' to print published data sizes in terminal          | true                       |


## Run

```bash
ros2 launch ntrip_client_ros ntrip_client_ros.launch.py
```












