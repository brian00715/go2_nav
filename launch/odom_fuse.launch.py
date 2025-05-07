import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )

    odom_fuse_node = Node(
        package="utils_pkg",
        executable="odom_fuse",
        name="odom_fuse_node",
        output="screen",
        parameters=[
            {
                "high_odom_topic": "/utlidar/robot_odom",
                "low_odom_topic": "/odom_lidar",
                "fused_odom_topic": "/odom",
                "use_kalman": False,
            }
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(odom_fuse_node)

    return ld
