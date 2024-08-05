#!/usr/bin/env python3

# Written by Simon Martineau
# ros2 run turtlebot4_testing lidar_noise --ros-args -p check_status:=1

# Parameters :
#   - check_status: 1 or 0, prints confirmation the node is running
#   - print_entry: 1 or 0, prints information about LiDAR topic
#   - no_mod: 1 or 0, does not add modifications to scan topic


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
        self.image_subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback ,10)
        self.image_publisher = self.create_publisher(LaserScan, "/scan_modified", 10)
        self.marker_publisher = self.create_publisher(Marker, "center_marker", 10)
        self.tf_marker_publisher = self.create_publisher(Marker, "tf_center_marker", 10)

        # Declaration of variables
        self.declare_parameter('check_status', 0)
        self.declare_parameter('print_entry', 0)
        self.declare_parameter('no_mod', 0)

        # Assignment of variable values
        self.check_status_val = self.get_parameter('check_status').get_parameter_value().integer_value
        self.print_entry_val = self.get_parameter('print_entry').get_parameter_value().integer_value
        self.no_mod_val = self.get_parameter('no_mod').get_parameter_value().integer_value

        # Initialize TF2 buffer and subscriber
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.check_status_val == 1:
            self.create_timer(1.0, self.timer_callback)  # timer_callback function will be called every 1s


    def entry_message_print(self, msg : LaserScan):
        self.get_logger().info(f"angle_min = {msg.angle_min}, angle_max = {msg.angle_max}, angle_increment = {msg.angle_increment}, scan_time = {msg.scan_time}, range_max = {msg.range_max}")


    


    def random_noise(self, msg : LaserScan):
        noise_level = 0.05  # Adjust this value to set the noise level
        noisy_ranges = []
        for r in msg.ranges:
            if not np.isinf(r):  # Only add noise to valid measurements
                noise = random.uniform(-noise_level, noise_level)
                noisy_ranges.append(r + noise)
            else:
                noisy_ranges.append(r)
        return noisy_ranges
    

    def dust_effect(self, msg : LaserScan):
        pass


    def simulate_circle(self):
        c_x, c_y = 0.0, 0.0  # center of circle in world rf
        radius = 1.0  # radius of the circle
        t = np.linspace(0, 2 * np.pi, 3)
        x = radius * np.cos(t) + c_x
        y = radius * np.sin(t) + c_y


    def particle_noise(self, msg : LaserScan):
        particle_probability = 0.01
        noisy_ranges = []
        for r in msg.ranges:
            if random.random() < particle_probability:  # A particle appears with particle_probability chance
                particle_distance = random.uniform(0, r)  # The particle will appear somewhere between the robot and the wall
                noisy_ranges.append(particle_distance)
            else:
                noisy_ranges.append(r)
        return noisy_ranges

        
    def circle_tryout(self, msg : LaserScan):
        circle_radius = 1.0 
        circle_ranges = []
        for r in msg.ranges:
            if not np.isinf(r):  # Only add noise to valid measurements
                circle_ranges.append(circle_radius)
            else:
                circle_ranges.append(r)
        return circle_ranges
    

    def lidar_callback(self, msg : LaserScan):
        # Everytime the robot gets a LiDAR message
        if self.print_entry_val == 1:
            self.entry_message_print(msg)

        if self.no_mod_val == 1:
            self.image_publisher.publish(msg)

        else:
            # Create a new LaserScan message with identical characteristics
            modified_msg = LaserScan()
            modified_msg.header = msg.header
            modified_msg.angle_min = msg.angle_min
            modified_msg.angle_max = msg.angle_max
            modified_msg.angle_increment = msg.angle_increment
            modified_msg.time_increment = msg.time_increment
            modified_msg.scan_time = msg.scan_time
            modified_msg.range_min = msg.range_min
            modified_msg.range_max = msg.range_max
            modified_msg.intensities = msg.intensities  # Keep intensities unchanged

            # Change LiDAR ranges for the new modified_msg
            # modified_msg.ranges = self.circle_tryout(msg)
            # modified_msg.ranges = self.random_noise(msg)
            modified_msg.ranges = self.particle_noise(msg)

            # Publish the modified scan
            self.image_publisher.publish(modified_msg)


    def timer_callback(self):
        self.get_logger().info("lidar_noise_node: Active")
        


def main(args=None):
    rclpy.init(args=args)
    node = LiDARNoiseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()













































