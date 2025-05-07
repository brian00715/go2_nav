#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header


class WaypointToNav2(Node):
    def __init__(self):
        super().__init__("waypoint_to_nav2")

        # Create subscription to /way_point topic
        self.subscription = self.create_subscription(PointStamped, "/way_point", self.waypoint_callback, 10)

        # Create action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # TF buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Waypoint to Nav2 node has been started")

    def waypoint_callback(self, msg):
        """Callback function for waypoint subscription"""
        self.get_logger().info(f"Received waypoint: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")

        # Convert waypoint to PoseStamped for Nav2
        nav_pose = self.waypoint_to_pose(msg)
        if nav_pose:
            # Send the navigation goal
            self.send_goal(nav_pose)

    def waypoint_to_pose(self, waypoint_msg):
        """Convert PointStamped waypoint to PoseStamped for navigation"""
        try:
            # Create a PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header = waypoint_msg.header

            # If waypoint is not in map frame, transform it
            if waypoint_msg.header.frame_id != "map":
                self.get_logger().info(f"Transforming waypoint from {waypoint_msg.header.frame_id} to map frame")
                try:
                    transform = self.tf_buffer.lookup_transform("map", waypoint_msg.header.frame_id, rclpy.time.Time())

                    # Create a temporary PoseStamped for the transformation
                    temp_pose = PoseStamped()
                    temp_pose.header = waypoint_msg.header
                    temp_pose.pose.position.x = waypoint_msg.point.x
                    temp_pose.pose.position.y = waypoint_msg.point.y
                    temp_pose.pose.position.z = waypoint_msg.point.z
                    temp_pose.pose.orientation.w = 1.0  # Default orientation

                    # Transform the pose
                    pose_stamped = tf2_geometry_msgs.do_transform_pose(temp_pose, transform)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
                    self.get_logger().error(f"TF transform error: {ex}")
                    return None
            else:
                # If already in map frame, just copy the point data
                pose_stamped.pose.position.x = waypoint_msg.point.x
                pose_stamped.pose.position.y = waypoint_msg.point.y
                pose_stamped.pose.position.z = waypoint_msg.point.z
                pose_stamped.pose.orientation.w = 1.0  # Default orientation

            # Ensure the final pose is in the map frame
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            return pose_stamped
        except Exception as e:
            self.get_logger().error(f"Error converting waypoint to pose: {e}")
            return None

    def send_goal(self, pose_stamped):
        """Send a navigation goal to Nav2"""
        # Wait for action server
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation action server not available")
            return

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f"Sending navigation goal: x={pose_stamped.pose.position.x}, y={pose_stamped.pose.position.y}"
        )

        # Send goal
        self.nav_to_pose_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Callback for goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            return

        self.get_logger().info("Navigation goal accepted")

        # Get result of navigation
        future = goal_handle.get_result_async()
        future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for getting the result of navigation"""
        result = future.result().result
        if result:
            self.get_logger().info("Navigation completed successfully")
        else:
            self.get_logger().warn("Navigation did not complete successfully")


def main(args=None):
    rclpy.init(args=args)

    waypoint_to_nav2 = WaypointToNav2()

    try:
        rclpy.spin(waypoint_to_nav2)
    except KeyboardInterrupt:
        waypoint_to_nav2.get_logger().info("Node stopped cleanly")
    except Exception as e:
        waypoint_to_nav2.get_logger().error(f"Error: {e}")
    finally:
        # Destroy the node explicitly
        waypoint_to_nav2.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
