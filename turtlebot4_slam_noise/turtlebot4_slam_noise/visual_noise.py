#!/usr/bin/env python3

# Written by Simon Martineau
# ros2 run turtlebot4_slam_noise visual_noise --ros-args -p check_status:=1 -p add_noise:=50 -p change_lighting:=0.5 -p add_blur:=10

"""
This code generates various visual noises to disrupt the camera feed and challenge the subsequent SLAM algorithm.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import time
import random



class VisualNoiseNode(Node):
    def __init__(self):
        super().__init__("visual_noise_node")
        self.image_subscriber = self.create_subscription(Image, "/camera/pi3/color/image_raw", self.image_callback ,10)
        self.image_publisher = self.create_publisher(Image, "/image_modified", 10)
        
        # Declaration of variables
        self.declare_parameters(
            namespace='',
            parameters=[
                ('check_status', rclpy.Parameter.Type.BOOL),
                ('add_noise', rclpy.Parameter.Type.INTEGER),
                ('change_lighting', rclpy.Parameter.Type.DOUBLE),
                ('add_blur', rclpy.Parameter.Type.INTEGER),
                ('add_water_drops', rclpy.Parameter.Type.INTEGER),
            ]
        )

        # Assignment of variable values
        self.check_status_val = self.get_parameter('check_status').get_parameter_value().bool_value
        self.add_noise_val = self.get_parameter('add_noise').get_parameter_value().integer_value
        self.change_lighting_val = self.get_parameter('change_lighting').get_parameter_value().double_value
        self.add_blur_val = self.get_parameter('add_blur').get_parameter_value().integer_value
        self.add_water_drops_val = self.get_parameter('add_water_drops').get_parameter_value().integer_value

        # Declarations for FPS measurement
        self.fps = 0
        self.last_time = time.time()

        if self.check_status_val == True:
            self.create_timer(3.0, self.timer_callback)  # timer_callback function will be called every 3s


    def add_noise(self, cv_image):
        # Example: Add some visual noise (random noise using OpenCV)
        noise = np.random.randn(cv_image.shape[0], cv_image.shape[1], cv_image.shape[2]) * self.add_noise_val
        noisy_image = cv_image + noise
        noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)
        return noisy_image


    def change_lighting(self, cv_image):
        # Adjust the brightness by multiplying with the change factor
        adjusted_image = cv2.convertScaleAbs(cv_image, alpha=self.change_lighting_val, beta=0)
        return adjusted_image


    def add_blur(self, cv_image):
        # Ensure the kernel size is odd
        if self.add_blur_val % 2 == 0:
            self.add_blur_val += 1

        # Apply Gaussian blur
        blurred_image = cv2.GaussianBlur(cv_image, (self.add_blur_val, self.add_blur_val), 0)
        return blurred_image


    def add_water_drops(self, cv_image):
        height, width, _ = cv_image.shape

        # Create a transparent overlay for water drops
        overlay = cv_image.copy()

        for _ in range(self.add_water_drops_val):
            # Randomly select position and size of the drop
            drop_x = random.randint(0, width)
            drop_y = random.randint(0, height)
            drop_radius = random.randint(5, 15)

            # Draw the water drop on the overlay
            cv2.circle(overlay, (drop_x, drop_y), drop_radius, (255, 255, 255, 50), -1)

        # Apply Gaussian blur to simulate water drop effect
        overlay = cv2.GaussianBlur(overlay, (15, 15), 0)

        # Blend the overlay with the original image
        water_dropped_image = cv2.addWeighted(overlay, 0.4, cv_image, 0.6, 0)
        return water_dropped_image
    

    def image_callback(self, msg: Image):
        # Measurement of fps in modified image
        current_time = time.time()
        self.fps = 1.0 / (current_time - self.last_time)
        self.last_time = current_time

        # Convert ROS Image message to numpy array for processing
        np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        # Convert numpy array to OpenCV format (BGR)
        cv_image = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)

        # Add noise
        if self.add_noise_val != 0:
            cv_image = self.add_noise(cv_image)

        # Change lighting
        if self.change_lighting_val != 1.0:
            cv_image = self.change_lighting(cv_image)

        # Add blur
        if self.add_blur_val != 1:
            cv_image = self.add_blur(cv_image)

        # Add water drops
        if self.add_water_drops_val != 0:
            cv_image = self.add_water_drops(cv_image)

        # Convert OpenCV image back to ROS image format
        modified_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        modified_msg = Image()
        modified_msg.header = msg.header  # Preserve the original header
        modified_msg.height = msg.height
        modified_msg.width = msg.width
        modified_msg.encoding = 'rgb8'
        modified_msg.is_bigendian = msg.is_bigendian
        modified_msg.step = msg.step
        modified_msg.data = modified_image_rgb.tobytes()

        # Publish the modified image
        self.image_publisher.publish(modified_msg)


    def timer_callback(self):
        self.get_logger().info("visual_noise_node: Active")
        self.get_logger().info(f'visual_noise_node: Active; FPS: {int(self.fps)}')


def main(args=None):

    rclpy.init(args=args)
    node = VisualNoiseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()