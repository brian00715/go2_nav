#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup

from unitree_go.msg import LowState


class JotStickButton:
    L1 = 2
    L2 = 32
    R1 = 1
    R2 = 16
    X = 4
    Y = 8
    A = 1
    B = 2
    UP = 16
    DOWN = 64
    LEFT = 128
    RIGHT = 32


class WaypointToNav2(Node):
    def __init__(self):
        super().__init__("waypoint_to_nav2")

        self.callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            PointStamped, "/way_point", self.waypoint_callback, 10, callback_group=self.callback_group
        )
        # self.low_state_sub = self.create_subscription(
        #     LowState, "/lowstate", self.lowstate_callback, 10, callback_group=self.callback_group
        # )
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.callback_group
        )
        # Track the current navigation goal
        self.current_goal_handle = None
        # Flag to indicate waiting for navigation result
        self.waiting_for_result = False

        self.get_logger().info("Waypoint to Nav2 node has been started")

    def lowstate_callback(self, msg):
        joystick = msg.wireless_remote
        if joystick[2] == JotStickButton.R2 + JotStickButton.L2:
            self.get_logger().info("R2+L2 pressed, canceling current goal")
            self.cancel_goals()

    def waypoint_callback(self, msg):
        """Callback function for waypoint subscription"""
        self.get_logger().info(f"Received new waypoint: x={msg.point.x}, y={msg.point.y}, z={msg.point.z}")

        # self.cancel_current_goal()

        nav_pose = self.waypoint_to_pose(msg)
        if nav_pose:
            # Send new navigation goal
            self.send_goal(nav_pose)
            pass

    def cancel_goals(self):
        """Cancel current navigation goal (if exists)"""
        if self.nav_to_pose_client and self.current_goal_handle is not None:
            self.get_logger().info("Cancelling all navigation goals")
            future = self.nav_to_pose_client.async_cancel_all_goals()
            future.add_done_callback(self._cancel_done_callback)
            self.current_goal_handle = None
            self.waiting_for_result = False

    def _cancel_done_callback(self, future):
        """Callback for when goal cancellation is complete"""
        try:
            cancel_response = future.result()
            if cancel_response:
                self.get_logger().info(f"Goals cancelled successfully: {len(cancel_response.goals_canceling)} goals")
            else:
                self.get_logger().info("No goals were cancelled")
        except Exception as e:
            self.get_logger().error(f"Exception while cancelling goals: {e}")

    def waypoint_to_pose(self, waypoint_msg):
        """Convert PointStamped waypoint to PoseStamped for navigation"""
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header = waypoint_msg.header


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
        # if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
        #     self.get_logger().error("Navigation server not available")
        #     return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f"Sending navigation goal: x={pose_stamped.pose.position.x}, y={pose_stamped.pose.position.y}"
        )

        # Set the flag to wait for result
        self.waiting_for_result = True

        # Send goal and save future to track the goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.current_goal_handle = send_goal_future

    def goal_response_callback(self, future):
        """Callback for goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Navigation goal rejected")
            self.current_goal_handle = None
            return

        self.get_logger().info("Navigation goal accepted")

        self.current_goal_handle = goal_handle

        future = goal_handle.get_result_async()
        future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Navigation result callback handling"""
        try:
            result = future.result()
            status = result.status

            # Clear waiting result flag and current goal handle
            self.waiting_for_result = False
            self.current_goal_handle = None

            # Handle result based on different status codes
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Navigation successfully completed")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info("Navigation has been canceled")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().warn("Navigation was aborted")
            else:
                self.get_logger().warn(f"Navigation did not complete successfully, status code: {status}")

        except Exception as e:
            self.get_logger().error(f"Error occurred while handling navigation result: {e}")
            self.waiting_for_result = False
            self.current_goal_handle = None

    def feedback_callback(self, feedback_msg):
        """Handle feedback during navigation"""
        feedback = feedback_msg.feedback
        # if hasattr(feedback, "distance_remaining") and feedback.distance_remaining % 1.0 < 0.1:
        #     self.get_logger().info(f"Distance to goal: {feedback.distance_remaining:.2f} meters")


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
