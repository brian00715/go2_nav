#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations


class OdomToTF(Node):
    """
    A ROS2 node that subscribes to an odometry topic and publishes the
    pose information as a transform in the tf tree.
    """

    def __init__(self):
        super().__init__("odom_to_tf")

        # Declare parameters with default values
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")

        # Get parameters
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        self.child_frame = self.get_parameter("child_frame").get_parameter_value().string_value

        # Create a transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a subscription to the odometry topic
        self.subscription = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        self.get_logger().info(
            f"Subscribed to {self.odom_topic}, publishing TF from {self.parent_frame} to {self.child_frame}"
        )

    def odom_callback(self, msg):
        """
        Callback function for the odometry topic.
        Converts the odometry message to a transform and broadcasts it.

        Args:
            msg (Odometry): The incoming odometry message
        """
        # Create a new transform message
        transform = TransformStamped()

        # Set the header information
        transform.header.stamp = msg.header.stamp
        # transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame

        # Set the child frame id
        transform.child_frame_id = self.child_frame

        # Set the transform translation from the odometry message
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Set the transform rotation from the odometry message
        transform.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """Main function to initialize the node and spin."""
    rclpy.init(args=args)

    odom_to_tf_node = OdomToTF()

    try:
        rclpy.spin(odom_to_tf_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        odom_to_tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
