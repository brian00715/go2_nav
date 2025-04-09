# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = "/home/unitree/dev_ws/src/go2_nav"
    launch_dir = os.path.join(bringup_dir, "launch")

    # Create the launch configuration variables
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace", default_value="false", description="Whether to apply a namespace to the navigation stack"
    )

    declare_slam_cmd = DeclareLaunchArgument("slam", default_value="False", description="Whether run a SLAM")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", default_value=os.path.join(bringup_dir, "maps", "e4a_3f.yaml"), description="Full path to map file to load"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the nav2 stack"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )

    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="True", description="Whether to start the simulator"
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub", default_value="False", description="Whether to start the robot state publisher"
    )

    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")

    urdf_file_name = "go2.urdf"
    urdf = os.path.join(get_package_share_directory("go2_nav"), "urdf", urdf_file_name)

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=remappings,
        arguments=[urdf],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_file}.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "default_bt_xml_filename": default_bt_xml_filename,
            "autostart": autostart,
        }.items(),
    )

    map_yaml = "/home/unitree/dev_ws/src/go2_nav/maps/e4a_3f.yaml"
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml},
                    {"frame_id": "map2"},],
    )
    lifecycle_nodes = ['map_server']
    lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    lidar2scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            # ("cloud_in", "/utlidar/cloud_deskewed"),
            # ("cloud_in", "/livox/points"),
            # ("cloud_in", "/livox/lidar"),
            # ("cloud_in", "/livox/pointcloud2"),
            ("cloud_in", "/aligned_points"),
            ("scan", "scan"),
        ],
        parameters=[
            {
                # "target_frame": "odom",
                # "target_frame": "base_link",
                # "target_frame": "livox_frame",
                "target_frame": "map2",
                "min_height": 0.2,
                "max_height": 0.4,
                "qos_overrides.cloud_in.reliability": "best_effort",
                "qos_overrides.scan.reliability": "reliable",
            }
        ],
        output="screen",
    )

    driver_file = "msg_MID360_launch.py"
    livox_ros_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("livox_ros_driver2"), "launch_ROS2", driver_file)]
        ),
        launch_arguments={
            "livox_ros_driver2_rviz": "false",
        }.items(),
    )

    static_tfs = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf1",
            arguments=["0", "0", "0", "0", "-0.16", "0", "map", "map2"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf2",
            arguments=["0","0","0","0","0","0", "map2", "odom"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf3",
            arguments=["0", "0", "-0.1", "0", "0", "0", "base_footprint", "base_link"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf4",
            arguments=["0.187", "0", "0.0803", "0", "0.2", "0", "base_link", "livox_frame"],
            # arguments=["0", "0", "0.2", "0", "-0.1", "0.2", "base_link", "livox_frame"],
            output="screen",
        ),
    ]
    odom2tf = Node(
        package="go2_nav",
        executable="odom2tf",
        name="odom2tf",
        output="screen",
        parameters=[
            {
                "freq": 50.0,
                # "odom_topic": "/utlidar/robot_odom",
                "odom_topic": "/odom",
                "parent_frame": "odom",
                "child_frame": "base_footprint",
            }
        ],
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    ld.add_action(lidar2scan_node)
    # ld.add_action(livox_ros_driver)
    ld.add_action(odom2tf)
    for static_tf in static_tfs:
        ld.add_action(static_tf)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager)

    return ld
