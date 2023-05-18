from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    sllidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen"
    )
    ld.add_action(sllidar_node)

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('zed_wrapper'), '/launch/zed2i.launch.py']
        )
    )
    ld.add_action(zed_wrapper_launch)

    zed_plot_node = Node(
        package="zed_plot",
        executable="zed_plot",
        name="zed_plot",
        remappings=[
            ("odom", "/zed2i/zed_node/odom"),
            ("pose", "/zed2i/zed_node/pose")
        ],
        parameters=[{
            "ros-args": "-r odom:=/zed2i/zed_node/odom -r pose:=/zed2i/zed_node/pose"
        }]
    )
    ld.add_action(zed_plot_node)

    map_to_myGrid = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_myGrid",
        arguments=["0", "0", "0", "0", "0", "0", "map", "myGrid"]
    )
    ld.add_action(map_to_myGrid)

    robotPoseFrame_to_laser_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robotPoseFrame_to_laser_frame",
        arguments=["0", "0", "0", "1.57", "0", "0", "robotPoseFrame", "laser_frame"]
    )
    ld.add_action(robotPoseFrame_to_laser_frame)

    return ld


if __name__ == '__main__':
    generate_launch_description()
