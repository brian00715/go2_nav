import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy.parameter
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


class DDSForward(Node):
    def __init__(self):
        super().__init__("dds_forward_node")
        self.forward_lidar = rclpy.parameter.Parameter("forward_lidar", rclpy.Parameter.Type.BOOL, False)

        self.odom_sub = self.create_subscription(Odometry, "/utlidar/robot_odom", self.odom_callback, 10)

        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1
        )

        if self.forward_lidar:
            self.get_logger().info("Will forward lidar topic")
            # self.lidar_sub = self.create_subscription(PointCloud2, "/pcl/voxel_grid", self.lidar_callback, 10)
            self.lidar_sub = self.create_subscription(PointCloud2, "/utlidar/cloud_deskewed", self.lidar_callback, 10)
            # self.lidar_pub = self.create_publisher(PointCloud2, "/utlidar/cloud_deskewed/baselink", 10)
            self.lidar_pub = self.create_publisher(PointCloud2, "/lidar", best_effort_qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("DDSForward has been started")

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def lidar_callback(self, msg: PointCloud2):
        msg.header = Header(frame_id="odom")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DDSForward()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
