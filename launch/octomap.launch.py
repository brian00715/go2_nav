from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server_node",
        output="screen",
        parameters=[
            {
                "frame_id": "odom",
                "resolution": 0.05,
                "sensor_model/max_range": -1.0,
                "sensor_model/hit": 1.0,
                "sensor_model/miss": 0.45,
                "sensor_model/min": 0.2,
                "sensor_model/max": 1.0,
                "occupancy_min_z": 0.1,
                "occupancy_max_z": 1.2,
            }
        ],
        remappings=[
            ("/cloud_in", "/utlidar/cloud_deskewed"),
        ],
    )
    tf_pub_node = Node(
        
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "/home/unitree/unitree_ros2/dev_ws/src/go2_nav/launch/config/default.rviz"],
    )
    return LaunchDescription(
        [
            rviz_node,
            octomap_node,
        ]
    )
