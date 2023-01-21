# zed-ros2-plot

This node plots position and heading data from the zed camera.

The starting point for the code is based on the [zed_pose_tutorial](https://github.com/stereolabs/zed-ros2-examples/tree/master/tutorials/zed_pose_tutorial)

### extra cpp-libraries
* [matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

### run

```bash
cd ~/ros2_ws
colcon build --packages-select zed_plot

ros2 launch zed_wrapper zed2i.launch.py
ros2 run zed_plot zed_plot --ros-args -r odom:=/zed2i/zed_node/odom -r pose:=/zed2i/zed_node/pose