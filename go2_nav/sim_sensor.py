#!/usr/bin/env python3

import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import tf2_ros
import transformations as tft
import yaml
from geometry_msgs.msg import TransformStamped
from numba import njit
from PIL import Image
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

TF_MAP1_TO_MAP2_PXL = np.array(
    [
        # [9.98952971e-01, 5.50242692e-02, -1.26839473e02],
        # [-5.55691195e-02, 9.98607084e-01, -8.74151475e01],
        # [-1.26192709e-06, 2.00148749e-08, 1.00000000e00],
        [9.98449836e-01, 1.00700868e-02, -5.22892986e00],
        [-1.16006916e-02, 1.00028136e00, -1.19019484e01],
        [-2.01684029e-06, 2.00056733e-07, 1.00000000e00],
    ]
)


@njit(fastmath=True)
def collision_check(x0, y0, x1, y1, gridmap, update_scan=False, lidar_scan=None, edge_pad_num=10):
    """
    Performs collision checking between two points and optionally updates a lidar scan.

    Parameters:
    -----------
    x0, y0: Starting point coordinates
    x1, y1: Ending point coordinates
    gridmap: 2D array representing the environment/ground truth
    update_scan: Boolean flag to determine if lidar_scan should be updated
    lidar_scan: 2D array to update with scan results (optional)
    edge_pad_num: Number of cells to continue checking after finding a collision (optional)

    Returns:
    --------
    collision: Boolean indicating if a collision was detected
    """
    collision = False
    x0 = int(round(x0))
    y0 = int(round(y0))
    x1 = int(round(x1))
    y1 = int(round(y1))

    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    error = dx - dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    dx *= 2
    dy *= 2

    collision_flag = 0

    while 0 <= x < gridmap.shape[1] and 0 <= y < gridmap.shape[0]:
        k = gridmap[y, x]

        # Update lidar scan if requested
        if update_scan and lidar_scan is not None:
            lidar_scan[y, x] = k

        # Check for collision
        if k == 1:
            collision = True
            collision_flag += 1
            if not update_scan or collision_flag >= edge_pad_num:
                break
        elif k == 127:
            collision = True
            if not update_scan:
                break
        elif update_scan and collision_flag > 0:
            break

        if x == x1 and y == y1:
            break

        if error > 0:
            x += x_inc
            error -= dy
        else:
            y += y_inc
            error += dx

    return collision


@njit(fastmath=True)
def sensor_work(robot_position, sensor_range, lidar_scan, ground_truth):
    """
    Input:
        lidar_scan: np.array of shape (height, width) current lidar_scan
        ground_truth: np.array of shape (height, width)
    Output:
        lidar_scan: updated lidar_scan
    """
    sensor_angle_inc = 0.5 / 180 * np.pi
    num_angles = int(2 * np.pi / sensor_angle_inc)
    x0 = robot_position[0]
    y0 = robot_position[1]
    for i in range(num_angles):
        sensor_angle = i * sensor_angle_inc
        x1 = x0 + np.cos(sensor_angle) * sensor_range
        y1 = y0 + np.sin(sensor_angle) * sensor_range
        collision_check(x0, y0, x1, y1, ground_truth, True, lidar_scan)
    return lidar_scan


class SimSensor(Node):
    def __init__(self, map_real, map_model, map_config):
        super().__init__("sim_sensor")

        self.map_origin = np.array(map_config["origin"])[:2]
        self.map_resolution = float(map_config["resolution"])
        self.map_real = np.array(Image.open(map_real))
        self.map_real = (self.map_real > 210).astype(np.uint8) * 254 + 1
        self.map_model = np.array(Image.open(map_model))
        self.map_model = (self.map_model > 210).astype(np.uint8) * 254 + 1
        self.map_real_size = self.map_real.shape
        self.map_model_size = self.map_model.shape

        self.robot_location = np.array([0, 0])
        self.robot_location_pxl = np.array([0, 0])

        self.lidar_scan = np.ones(self.map_real_size) * 127

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Map to base_link TF listener node started")

        # Create a separate, faster timer just for GUI updates
        self.gui_timer = self.create_timer(0.01, self.gui_callback)
        self.display_img = None
        cv2.namedWindow("lidar_scan", cv2.WINDOW_NORMAL)

    def gui_callback(self):
        if self.display_img is not None:
            cv2.imshow("lidar_scan", self.display_img)
            cv2.waitKey(1)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation
            # self.get_logger().info(f"pos | x: {translation.x:.2f} | y: {translation.y:.2f}")

            coord_phi = np.array([translation.x, translation.y])
            coord_map1_phi = coord_phi - self.map_origin
            coord_map1_phi = [coord_map1_phi[0], coord_map1_phi[1], 1]
            # Note: In image coordinates, origin is at top-left, y-axis points down
            coord_map1_pxl = np.array(
                [
                    coord_map1_phi[0] / self.map_resolution,
                    self.map_real_size[0] - coord_map1_phi[1] / self.map_resolution,  # Flip y-axis
                ]
            )

            coord_map2_pxl = np.dot(TF_MAP1_TO_MAP2_PXL, [coord_map1_pxl[0], coord_map1_pxl[1], 1])[0:2]
            coord_map2_pxl = coord_map2_pxl.astype(np.uint16)
            # self.get_logger().info(f"pos | map1: {coord_map1_phi[:2]} | coord_map2_pxl: {coord_map2_pxl}")

            self.robot_location_pxl = coord_map2_pxl
            self.lidar_scan = sensor_work(self.robot_location_pxl, 100, self.lidar_scan, self.map_model)

            ratio = 0.8
            img_show = self.lidar_scan.copy()
            cv2.circle(img_show, tuple(self.robot_location_pxl), 5, (0, 0, 255), -1)
            img_scale = cv2.resize(img_show, (0, 0), fx=ratio, fy=ratio)
            img_scale = img_scale.astype(np.uint8)
            img_scale = cv2.cvtColor(img_scale, cv2.COLOR_GRAY2BGR)
            self.display_img = img_scale

        except Exception as e:
            self.get_logger().warning(f"Error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    mappath = "/home/unitree/dev_ws/src/go2_nav/maps/"
    mapname = "e4a_3f"
    map_model = os.path.join(mappath, mapname + "_model.png")
    map_real = os.path.join(mappath, mapname + "_adjust.png")
    map_config = yaml.safe_load(open(os.path.join(mappath, mapname + "_adjust.yaml")))

    if 1:
        node = SimSensor(map_real, map_model, map_config)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    if 0:
        output_path = "/home/unitree/dev_ws/"
        ground_truth = np.array(Image.open(map_real))
        ground_truth = (ground_truth > 210).astype(np.uint8) * 254 + 1
        map_gt_size = ground_truth.shape
        plt.imsave(output_path + "/ground_truth.png", ground_truth, cmap="gray")

        robot_location = np.array([300, 600])
        lidar_scan = np.ones(map_gt_size) * 127
        lidar_scan = sensor_work(robot_location, 100, lidar_scan, ground_truth)
        plt.imsave(output_path + "/lidar_scan.png", lidar_scan, cmap="gray")


if __name__ == "__main__":
    main()
