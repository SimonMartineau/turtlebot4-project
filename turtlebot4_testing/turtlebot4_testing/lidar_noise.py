# Written By Simon Martineau

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
import numpy as np
import random


class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.obstacle_points_list = [] # Initialise obstacle points list in the map frame
        self.transformed_points_list = [] # Initialise obstacle points list in the base_link frame
        self.obstacle_dist_list = [] # List of euclidian distance to obstacle points in base_link frame
        self.obstacle_angle_list = [] # List of angles to obstacle points in base_link frame

        self.image_subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback ,10)
        self.image_publisher = self.create_publisher(LaserScan, "/scan_modified", 10)
        self.lidar_list_offset = 809
        
        # Schedule the transformation after 2 seconds to ensure tf2 has time to receive transforms
        # self.timer = self.create_timer(2.0, self.transform_point)
        # self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)

        
    def create_circle_obstacle(self):
        # Reset the obstacle points lists in both frames
        self.obstacle_points_list = []
        self.transformed_points_list = []
        self.obstacle_dist_list = []
        self.obstacle_angle_list = []

        # Create virtual obstacle points
        n_points = 20
        for i in range (n_points):
            contour_point = PointStamped()
            contour_point.header.frame_id = 'map'
            contour_point.header.stamp = self.get_clock().now().to_msg()
            contour_point.point.x = 0.3 * np.cos(i/n_points * 2*np.pi) + 1.0
            contour_point.point.y = 0.3 * np.sin(i/n_points * 2*np.pi) - 1.0
            contour_point.point.z = 0.0
            self.obstacle_points_list.append(contour_point)

            # try:
            #    map_marker = self.create_marker(contour_point, 'map', 0)
            #    self.marker_publisher.publish(map_marker)
            # except:
            #    pass
        

    def create_dust_points(self):
        # This dust cloud is a noisy circle to give illusion of moving dust
        # Reset the obstacle points lists in both frames
        self.obstacle_points_list = []
        self.transformed_points_list = []
        self.obstacle_dist_list = []
        self.obstacle_angle_list = []

        # Create virtual obstacle points
        n_points = 300
        radius = 0.3
        for i in range (n_points):
            contour_point = PointStamped()
            contour_point.header.frame_id = 'map'
            contour_point.header.stamp = self.get_clock().now().to_msg()
            contour_point.point.x = (radius + random.uniform(0.0, 0.2)) * np.cos(i/n_points * 2*np.pi) + 1.0
            contour_point.point.y = (radius + random.uniform(0.0, 0.2)) * np.sin(i/n_points * 2*np.pi) - 1.0
            contour_point.point.z = 0.0
            self.obstacle_points_list.append(contour_point)

            # try:
            #    map_marker = self.create_marker(contour_point, 'map', 0)
            #    self.marker_publisher.publish(map_marker)
            # except:
            #    pass


    def transform_point(self):
        self.create_dust_points()
        try:
            # Lookup the transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())

            # Transform the points
            for map_point in self.obstacle_points_list:
                transformed_point = do_transform_point(map_point, transform)
                self.transformed_points_list.append(transformed_point)
                dist_to_obstacle = np.sqrt(transformed_point.point.x**2 + transformed_point.point.y**2) # Calculate distance between the robot and the obstacle point
                angle_to_obstacle = np.arctan2(transformed_point.point.y, transformed_point.point.x) + np.pi # Returns angle between 0 and 2*pi
                self.obstacle_dist_list.append(dist_to_obstacle) 
                self.obstacle_angle_list.append(angle_to_obstacle)
                
                for i in range(1, len(self.obstacle_dist_list)-1):
                    if self.obstacle_dist_list[i] > self.obstacle_dist_list[i-1]*1.01 and self.obstacle_dist_list[i] > self.obstacle_dist_list[i+1]*1.01:
                        self.obstacle_dist_list[i] = (self.obstacle_dist_list[i+1] + self.obstacle_dist_list[i-1])/2

                # self.get_logger().info(f"Transformed point: {transformed_point.point.x}, {transformed_point.point.y}")
                # self.get_logger().info(f"Transformed point distance: {dist_to_obstacle}")
                # self.get_logger().info(f"Transformed point angle: {angle_to_obstacle}")
                # Points on the other side of the obstacle might be sent to LiDAR ranges list, this reduces issue

        except Exception as e:
            self.get_logger().error(f"Could not transform point: {str(e)}")

        try:
            for id_num, map_point in enumerate(self.obstacle_points_list):
                map_marker = self.create_marker(map_point, 'map', id_num)
                self.marker_publisher.publish(map_marker)

            for id_num, transformed_point in enumerate(self.transformed_points_list):
                transformed_marker = self.create_marker(transformed_point, 'base_link', id_num)
                self.marker_publisher.publish(transformed_marker)
        except:
            pass


    def create_marker(self, point_stamped, frame_id, marker_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_noise"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0 if frame_id == 'map' else 0.0
        marker.color.b = 1.0 if frame_id == 'base_link' else 0.0
        return marker
    

    def lidar_mod(self, msg : LaserScan, modified_range, modified_intensities):
        # Modification for obstacle points
        for ang_index, angles in enumerate(self.obstacle_angle_list):
            obstacle_index = int((len(msg.ranges)-1) * angles / (2*np.pi))
            # self.get_logger().info(f"obstacle_index: {obstacle_index}")
            # self.get_logger().info(f"lidar ranges: {len(msg.ranges)}")
            # self.get_logger().info(f"index: {obstacle_index + self.lidar_list_offset - len(msg.ranges)}")

            # We apply an offset because angle and the LiDAR ranges list are not syncronised
            if obstacle_index + self.lidar_list_offset > len(msg.ranges) - 1:
                if modified_range[obstacle_index + self.lidar_list_offset - len(msg.ranges)] > self.obstacle_dist_list[ang_index]:  # See if virtual obstacle is closer than wall
                    modified_range[obstacle_index + self.lidar_list_offset - len(msg.ranges)] = self.obstacle_dist_list[ang_index]
                    modified_intensities[obstacle_index + self.lidar_list_offset - len(msg.ranges)] = 1.5  # Dust has lidar intensity between 1 and 2 

            else:
                if modified_range[obstacle_index + self.lidar_list_offset] > self.obstacle_dist_list[ang_index]:  # See if virtual obstacle is closer than wall
                    modified_range[obstacle_index + self.lidar_list_offset] = self.obstacle_dist_list[ang_index]
                    modified_intensities[obstacle_index + self.lidar_list_offset] = 1.5  # Dust has lidar intensity between 1 and 2 
        return modified_range, modified_intensities
    

    def lidar_callback(self, msg : LaserScan):
        self.transform_point()

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

        # Change LiDAR ranges and intensities for the new modified_msg

        modified_msg.ranges, modified_msg.intensities = self.lidar_mod(msg, msg.ranges, modified_msg.intensities)  # If you want changes
        #modified_msg.ranges = msg.ranges  # If you don't want changes
        self.image_publisher.publish(modified_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()















































