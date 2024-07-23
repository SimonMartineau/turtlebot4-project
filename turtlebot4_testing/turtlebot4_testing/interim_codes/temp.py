#!/usr/bin/env python3

# Written by Simon Martineau
# ros2 run turtlebot4_testing lidar_noise

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import tf2_geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


class LiDARNoiseNode(Node):
    def __init__(self):
        super().__init__("lidar_noise_node")

        # Initialize TF2 buffer and subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def transform_tryouts(self):
        # Create a point in the 'map' frame
        point_in_map = PointStamped()
        point_in_map.header.frame_id = 'map'
        point_in_map.header.stamp = self.get_clock().now().to_msg()
        point_in_map.point.x = 0.0
        point_in_map.point.y = 0.0
        point_in_map.point.z = 0.0

        # Transform the point in the 'odom' map
        try:
            transformed_point = self.tf_buffer.transform(point_in_map, 'base_link')
            self.get_logger().info(f"tf points = {transformed_point}")
        except Exception as ex:
            self.get_logger().info(f"{ex}")


    def lidar_callback(self, msg : LaserScan):
        # Everytime the robot gets a LiDAR message
        self.transform_tryouts()


def main(args=None):
    rclpy.init(args=args)
    node = LiDARNoiseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


