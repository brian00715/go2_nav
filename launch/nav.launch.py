import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = "/home/unitree/dev_ws/src/go2_nav"
    launch_dir = os.path.join(nav_dir, "launch")

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace", default_value="false", description="Whether to apply a namespace to the navigation stack"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true"
    )
    declare_nav_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav_dir, "config", "nav2_params.yaml"),
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
        default_value=os.path.join(nav_dir, "rviz", "nav2_default_view.rviz"),
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
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz.launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={"namespace": "", "use_namespace": "False", "rviz_config": rviz_config_file}.items(),
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "default_bt_xml_filename": default_bt_xml_filename,
            "autostart": autostart,
        }.items(),
    )

    # map_yaml = "/home/unitree/dev_ws/src/go2_nav/maps/e4a_3f.yaml"
    # map_yaml = "/home/unitree/dev_ws/src/go2_nav/maps/e4a_3f_nav.yaml"
    map_yaml = "/home/unitree/dev_ws/src/go2_nav/maps/tlab_6f_nav.yaml"
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml},
            {"frame_id": "map"},
        ],
    )
    lifecycle_nodes = ["map_server"]
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, {"autostart": autostart}, {"node_names": lifecycle_nodes}],
    )

    lidar2scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            # ("cloud_in", "/utlidar/cloud_deskewed"),
            # ("cloud_in", "/utlidar/cloud"),
            # ("cloud_in", "/livox/points"),
            ("cloud_in", "/livox/lidar"),
            # ("cloud_in", "/livox/pointcloud2"),
            # ("cloud_in", "/aligned_points"),
            ("scan", "scan"),
        ],
        parameters=[
            {
                # "target_frame": "map",
                # "target_frame": "utlidar_lidar",
                "target_frame": "livox_frame",
                # "target_frame": "base_link",
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
            arguments=["0", "0", "0", "0", "0.16", "0", "map_real", "map"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf2",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf3",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="baselink2vehicle",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "vehicle"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf4",
            # arguments=["0.187", "0", "0.0803", "0", "0.16", "0", "base_link", "livox_frame"],
            arguments=["-0.187", "0", "-0.0803", "0", "-0.16", "0", "livox_frame", "base_link"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf5",
            arguments=["0.289", "0", "-0.047", "3.142", "0.263", "3.142", "base_link", "utlidar_lidar"],
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
                "freq": -1,
                # "odom_topic": "/utlidar/robot_odom",
                "odom_topic": "/odom",
                "parent_frame": "map",
                # "child_frame": "base_footprint",
                # "child_frame": "base_link",
                "child_frame": "livox_frame",
            }
        ],
    )

    waypoint2nav2 = Node(
        package="go2_nav",
        executable="waypoint2nav2",
        name="waypoint2nav2",
        output="screen",
    )

    ekf_params_file = "/home/unitree/dev_ws/src/go2_nav/config/ekf_params.yaml"
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file],
        remappings=[
            ("/odometry/filtered", "/odom"),
        ],
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
                "pub_tf": True,
                "tf_from": "odom",
                "tf_to": "livox_frame",
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    for static_tf in static_tfs:
        ld.add_action(static_tf)
    # ld.add_action(lidar2scan_node)
    # ld.add_action(livox_ros_driver)
    # ld.add_action(odom2tf)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager)

    ld.add_action(waypoint2nav2)
    # ld.add_action(ekf_node)
    ld.add_action(odom_fuse_node)

    return ld
