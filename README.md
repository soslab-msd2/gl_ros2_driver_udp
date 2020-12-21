# GL-3 (Mechanical scanning 2D LiDAR)

## Guide
* Installation
```
$ cd ${ROS2 workspace}/src
$ git clone https://github.com/soslab-project/gl_ros2_driver_udp.git
$ cd $(ROS2 workspace)
$ colcon build --symlink-install
```
- Set local IP as `10.110.1.3`
- Run GL-3 publisher node
```
$ ros2 launch gl_ros2_driver_udp gl_ros2_driver_udp.py
```
- Run GL-3 publisher node with RViz
```
$ ros2 launch gl_ros2_driver_udp view_gl_ros2_driver_udp.py
```
- Change setting parameters in `gl_ros2_driver_udp/launch/gl_ros2_driver_udp.py`

## Published Topics
- _scan_ (sensor_msgs/LaserScan): it publishes scan topic from the laser.

## Test environment
- ROS2 Dashing Diademata
- Ubuntu 18.04 LTS

## Application demo
- [GL-3, Demo] 2D LiDAR, Mapping (https://youtu.be/AfsqlU_f-Go)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (https://youtu.be/_HBWe95GqXM)
- [GL-3, Demo] Create 3D Point Cloud with 2D LiDAR (pulse-width version) (https://youtu.be/q4MeeS9eP4c)
- [GL-3, Demo] Human Tracking Demo of Multiple Mobile Robots (https://youtu.be/ZzEvm8gMPaA)
- [GL-3, Demo] 2D LiDAR Object Detection (https://youtu.be/V29QzU9AcQo)
