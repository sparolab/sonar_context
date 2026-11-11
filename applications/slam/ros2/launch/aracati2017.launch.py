"""Aracati 2017 on ROS 2: play a ROS 2 bag with the sonar images and odometry poses (DISO's /direct_sonar/pose,
recorded with the ROS 1 launch and converted with rosbags-convert) into the Sonar Context SLAM.

    ros2 launch sonar_context_slam aracati2017.launch.py bag:=/data/aracati_diso save_dir:=/out/ros2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag, save_dir, rate = LaunchConfiguration("bag"), LaunchConfiguration("save_dir"), LaunchConfiguration("rate")
    return LaunchDescription([
        DeclareLaunchArgument("bag"),
        DeclareLaunchArgument("save_dir", default_value=""),
        DeclareLaunchArgument("rate", default_value="1.0"),
        Node(package="sonar_context_slam", executable="sonar_context_slam_node", name="sonar_context_slam",
             output="screen",
             parameters=[PathJoinSubstitution([FindPackageShare("sonar_context_slam"), "config", "aracati2017.yaml"]),
                         {"save_dir": save_dir}]),
        ExecuteProcess(cmd=["ros2", "bag", "play", bag, "-r", rate], output="log"),
    ])
