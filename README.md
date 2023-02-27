# zed-ros2-plot

This node plots position and heading data from the zed camera.

The starting point for the code is based on the [zed_pose_tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_pose_tutorial)

### extra cpp-libraries
* [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

### run

```bash
cd ~/ros2_ws
colcon build --packages-select zed_plot

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
