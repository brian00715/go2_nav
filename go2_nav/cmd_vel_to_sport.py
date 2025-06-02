#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from unitree_api.msg import Request
from scripts.sport_client import SportClient
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


class CmdVelToSportNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_sport_node")

        # get param of ros version
        self.declare_parameter("ros_version", "ros1")
        self.ros_version = self.get_parameter("ros_version").get_parameter_value().string_value
        self.get_logger().info(f"ros_version: {self.ros_version}")

        self.sport_client = SportClient()

        self.sport_pub = self.create_publisher(Request, "/api/sport/request", 10)

        self.cmd_vel_sub = self.create_subscription(
            TwistStamped if self.ros_version == "ros1" else Twist, "cmd_vel", self.cmd_vel_callback, 10
        )
        self.low_state_sub = self.create_subscription(LowState, "/lowstate", self.lowstate_callback, 10)

        self.pause_pub = False

        self.get_logger().info("CmdVel to Sport node initialized")

    def lowstate_callback(self, msg):
        joystick = msg.wireless_remote
        # if joystick[2] == JotStickButton.R2 + JotStickButton.L2:
        if joystick[2] == JotStickButton.R2:
            self.get_logger().info("R2 pressed, pauseing publishing")
            self.pause_pub = True
            sport_msg = self.sport_client.move(0, 0, 0)
            self.sport_pub.publish(sport_msg)
        # if joystick[3] == JotStickButton.A:
            # self.get_logger().info("A pressed, resuming publishing")
        if joystick[2] == JotStickButton.R1:
            self.get_logger().info("R1 pressed, resuming publishing")
            self.pause_pub = False

    def cmd_vel_callback(self, msg):
        if self.ros_version == "ros1":  # twistStamped
            x = msg.twist.linear.x
            y = msg.twist.linear.y
            z = msg.twist.angular.z
        else:  # twist
            x = msg.linear.x
            y = msg.linear.y
            z = msg.angular.z

        if (x != 0.0 or y != 0.0 or z != 0.0) and not self.pause_pub:
            # self.get_logger().info(f"Moving with velocities: x={x}, y={y}, z={z}")
            sport_msg = self.sport_client.move(x, y, z)
            self.sport_pub.publish(sport_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
