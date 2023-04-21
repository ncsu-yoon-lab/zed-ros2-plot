# zed-ros2-plot
![ros2-build](https://github.com/ncsu-yoon-lab/zed-ros2-plot/actions/workflows/ros2-build.yml/badge.svg)

This node plots position and heading data from the zed camera.

The starting point for the code is based on the [zed_pose_tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_pose_tutorial)

### extra cpp-libraries
* [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

### run

```bash
cd ~/ros2_ws
colcon build --packages-select zed_plot

# foxglove setup
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# rviz sample
ros2 launch zed_display_rviz2 display_zed2.launch.py
# transforms
ros2 run tf2_ros static_transform_publisher 0 0 0 1.57 0 -0.27 base_link laser_frame
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map myGrid
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom robotPose
# record
ros2 bag record /scan /zed2i/zed_node/pose /tf /tf_static

# start wolfwagen
ros2 run joy joy_node --ros-args -p autorepeat_rate:=0.0
./xbox_controller.py
./pwm_genV3.py


ros2 run sllidar_ros2 sllidar_node
ros2 launch zed_wrapper zed2i.launch.py
ros2 run zed_plot zed_plot --ros-args -r odom:=/zed2i/zed_node/odom -r pose:=/zed2i/zed_node/pose
```

### setup port
```bash
# check if linux recognizes the lidar usb serial convertor
lsusb
ls -l /dev |grep ttyUSB

# set write permission to device 0 (could be a different number)
# which is the usb serial convertor for lidar
sudo chmod 666 /dev/ttyUSB0
```
