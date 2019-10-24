# Hesai Pandar40P 

This repository includes the ROS Driver for the Pandar40p LiDAR sensor manufactured by Hesai Technology.

[![Build Status](https://travis-ci.org/amc-nu/Pandar40p_ros.svg?branch=master)](https://travis-ci.org/amc-nu/Pandar40p_ros)

## How to Build

### Install `catkin_tools`

```
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

### Compile

1. Create ROS Workspace. i.e. `rosworkspace`
```
$ mkdir -p rosworkspace/src
$ cd rosworkspace/src
```

2. Clone recursively this repository.
3. Install required dependencies with the help of `rosdep` 
```
$ cd ..
$ rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO 
```
4. Compile
```
$ catkin config --install
$ catkin build --force-cmake
```

## How to Launch

1. While in the `rosworkspace` directory.
```
$ source install/setup.bash
$ roslaunch pandar pandar40p_driver.launch
```
2. The driver will publish a PointCloud message in the topic.
```
/pandar40p/sensor/pandar40p/hesai40/PointCloud2
```
3. Open Rviz and add display by topic.
4. Change fixed frame to match the one in the launch file (Default `hesai40`)

## Available parameters in the Launch file

|Parameter | Default Value|
|---------|---------------|
|pandar40p_ip | 192.168.20.51|
|lidar_recv_port |2368|
|gps_recv_port  |10110|
|start_angle |135|
|timezone| 8|
|frame_id |hesai40|
|lidar_topic |/pandar40p/sensor/pandar40p/hesai40/PointCloud2|