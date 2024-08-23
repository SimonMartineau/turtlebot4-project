# Written By Simon Martineau
# ros2 run turtlebot4_slam_noise dust_noise --ros-args -p dust_transparency:=0.9

"""
This code generates a simulated dust cloud on the map which the LiDAR will detect and relay to NAV2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import numpy as np
import random



class SimulatedDustCloud(Node):
    def __init__(self):
        super().__init__('dust_noise_node')
        self.tf_buffer = Buffer()  # This buffer is used to transform the zone's corner points into the robot frame
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_buffer2 = Buffer()  # This buffer is used to transform the potential scans into the map frame
        self.tf_listener2 = TransformListener(self.tf_buffer2, self)

        self.zone_points_list = [] # Initialise obstacle points list in the map frame
        self.transformed_zone_points_list = [] # Initialise obstacle points list in the base_link frame
        self.dist_to_zone = 10
        self.angle_to_zone = []
        
        self.image_subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback ,10)  # Input LiDAR scan topic
        self.image_publisher = self.create_publisher(LaserScan, "/scan_modified", 10)    # Output LiDAR scan topic
        self.lidar_list_offset = 809  # Offset to match LiDAR values lists with robot's angles
        self.inside_dust_zone = False
        
        self.marker_publisher = self.create_publisher(Marker, '/dust_zone_markers', 10)

        # Declaration of variables
        self.declare_parameters(
            namespace='',
            parameters=[
                ('check_status', rclpy.Parameter.Type.BOOL),
                ('dust_transparency', rclpy.Parameter.Type.DOUBLE),
                ('dust_zone_center_x', rclpy.Parameter.Type.DOUBLE),
                ('dust_zone_center_y', rclpy.Parameter.Type.DOUBLE),
                ('dust_zone_length', rclpy.Parameter.Type.DOUBLE),
                ('dust_zone_width', rclpy.Parameter.Type.DOUBLE),
                ('dust_zone_angle', rclpy.Parameter.Type.DOUBLE),
                ('dust_absorption_dist_limit', rclpy.Parameter.Type.DOUBLE),
                ('dust_absorption_limit', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        # Assignment of variable values
        self.check_status_val = self.get_parameter('check_status').get_parameter_value().integer_value
        self.dust_transparency_val = self.get_parameter('dust_transparency').get_parameter_value().double_value
        self.dust_zone_center_x_val = self.get_parameter('dust_zone_center_x').get_parameter_value().double_value
        self.dust_zone_center_y_val = self.get_parameter('dust_zone_center_y').get_parameter_value().double_value
        self.dust_zone_length_val = self.get_parameter('dust_zone_length').get_parameter_value().double_value
        self.dust_zone_width_val = self.get_parameter('dust_zone_width').get_parameter_value().double_value
        self.dust_zone_angle_val = self.get_parameter('dust_zone_angle').get_parameter_value().double_value
        self.dust_absorption_dist_val = self.get_parameter('dust_absorption_dist_limit').get_parameter_value().double_value
        self.dust_absorption_val = self.get_parameter('dust_absorption_limit').get_parameter_value().double_value

        if self.check_status_val == 1.0:
            self.create_timer(3.0, self.timer_callback)  # timer_callback function will be called every 3s

        self.create_dust_zone()  # Creates the obstacle points list in the map frame


    def create_dust_zone(self):
        # This function creates the 4 points of the rectangle simulated dust cloud.
        # Create the corners of the dust zone rectangle according to the parameters and adds points to self.zone_points_list
        zone_point_1 = PointStamped()
        zone_point_1.header.frame_id = 'map'
        zone_point_1.header.stamp = self.get_clock().now().to_msg()
        zone_point_1.point.x = self.dust_zone_center_x_val + 0.5*(self.dust_zone_length_val*np.cos(self.dust_zone_angle_val) + self.dust_zone_width_val*np.sin(self.dust_zone_angle_val))
        zone_point_1.point.y = self.dust_zone_center_y_val + 0.5*(self.dust_zone_width_val*np.cos(self.dust_zone_angle_val) - self.dust_zone_length_val*np.sin(self.dust_zone_angle_val))
        zone_point_1.point.z = 0.0
        self.zone_points_list.append(zone_point_1)

        zone_point_2 = PointStamped()
        zone_point_2.header.frame_id = 'map'
        zone_point_2.header.stamp = self.get_clock().now().to_msg()
        zone_point_2.point.x = self.dust_zone_center_x_val + 0.5*(self.dust_zone_length_val*np.cos(self.dust_zone_angle_val) - self.dust_zone_width_val*np.sin(self.dust_zone_angle_val))
        zone_point_2.point.y = self.dust_zone_center_y_val + 0.5*(-self.dust_zone_width_val*np.cos(self.dust_zone_angle_val) - self.dust_zone_length_val*np.sin(self.dust_zone_angle_val))
        zone_point_2.point.z = 0.0
        self.zone_points_list.append(zone_point_2)

        zone_point_3 = PointStamped()
        zone_point_3.header.frame_id = 'map'
        zone_point_3.header.stamp = self.get_clock().now().to_msg()
        zone_point_3.point.x = self.dust_zone_center_x_val + 0.5*(-self.dust_zone_length_val*np.cos(self.dust_zone_angle_val) - self.dust_zone_width_val*np.sin(self.dust_zone_angle_val))
        zone_point_3.point.y = self.dust_zone_center_y_val + 0.5*(-self.dust_zone_width_val*np.cos(self.dust_zone_angle_val) + self.dust_zone_length_val*np.sin(self.dust_zone_angle_val))
        zone_point_3.point.z = 0.0
        self.zone_points_list.append(zone_point_3)

        zone_point_4 = PointStamped()
        zone_point_4.header.frame_id = 'map'
        zone_point_4.header.stamp = self.get_clock().now().to_msg()
        zone_point_4.point.x = self.dust_zone_center_x_val + 0.5*(-self.dust_zone_length_val*np.cos(self.dust_zone_angle_val) + self.dust_zone_width_val*np.sin(self.dust_zone_angle_val))
        zone_point_4.point.y = self.dust_zone_center_y_val + 0.5*(self.dust_zone_width_val*np.cos(self.dust_zone_angle_val) + self.dust_zone_length_val*np.sin(self.dust_zone_angle_val))
        zone_point_4.point.z = 0.0
        self.zone_points_list.append(zone_point_4)


    def transform_points(self):
        # This function transforms the obstacle points from the map frame to the robot's frame and publishes markers
        self.transformed_zone_points_list = []  # Reinitialise the list

        # The transform doesn't always work when data is inaccessible, so try/except is used
        try:
            # Lookup the transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())

            # Transform the corner points of the dust zone into the robot frame
            for map_point in self.zone_points_list:
                self.transformed_zone_points_list.append(do_transform_point(map_point, transform))
        except Exception as e:
            self.get_logger().error(f"Could not transform point: {str(e)}")

        # Here we publish markers on the obstacle points in the robot's frame and in the map frame to test if everything works
        try:
            # Publish the markers in the map frame
            for id_num, map_point in enumerate(self.zone_points_list):
                map_marker = self.create_marker(map_point, 'map', id_num)
                self.marker_publisher.publish(map_marker)

            # Publish the markers in the robot frame
            for id_num, transformed_point in enumerate(self.transformed_points_list):
                transformed_marker = self.create_marker(transformed_point, 'base_link', id_num)
                self.marker_publisher.publish(transformed_marker)
        except:
            pass


    def lidar_modifications(self, modified_msg : LaserScan):
        # This function modifies the LiDAR measurements and intensities so the obstacle appears where the zone is located.
        zone_angle_min_index = int((len(modified_msg.ranges)-1) * self.angle_to_zone[0]/(2*np.pi))
        zone_angle_max_index = int((len(modified_msg.ranges)-1) * self.angle_to_zone[1]/(2*np.pi))

        arc_1 = self.angle_to_zone[1] - self.angle_to_zone[0]  # If the zone is in front or besides the robot
        arc_2 = 2*np.pi - self.angle_to_zone[1] + self.angle_to_zone[0]  # If the zone is behind the robot, the skip from 360 to 0 happens and a correction is applied

        # We must take the smallest arc value
        if arc_1 < arc_2:
            for lidar_index in range(zone_angle_min_index, zone_angle_max_index):
                if lidar_index + self.lidar_list_offset < len(modified_msg.ranges):
                    self.apply_lidar_noise(modified_msg, lidar_index + self.lidar_list_offset)
                else:
                    self.apply_lidar_noise(modified_msg, lidar_index + self.lidar_list_offset - len(modified_msg.ranges))
        else:   
            for lidar_index in range(0, zone_angle_min_index):
                if lidar_index + self.lidar_list_offset < len(modified_msg.ranges):
                    self.apply_lidar_noise(modified_msg, lidar_index + self.lidar_list_offset)
                else:
                    self.apply_lidar_noise(modified_msg, lidar_index + self.lidar_list_offset - len(modified_msg.ranges))

            for lidar_index in range(zone_angle_max_index, len(modified_msg.ranges)-1):
                if lidar_index + self.lidar_list_offset < len(modified_msg.ranges):   
                    self.apply_lidar_noise(modified_msg, lidar_index + self.lidar_list_offset)
                else:
                    self.apply_lidar_noise(modified_msg, lidar_index + self.lidar_list_offset - len(modified_msg.ranges))


    def apply_lidar_noise(self, modified_msg : LaserScan, index_value):
        # This function applies the dust noise to the LiDAR measurement according to experimental data as well as tuning for user.

        # If the dust within 55cm of the robot, it absorbs the LiDAR measurements with 90% chance
        if self.dist_to_zone < self.dust_absorption_dist_val:
            if random.random() < self.dust_absorption_val:
                #self.get_logger().info(f"absorbed")
                modified_msg.ranges[index_value] = np.inf  # The dust absorbes the LiDAR scan according to experiments

        else:
            # If the robot is further away than 55cm, the LiDAR will detect dust with parameter chance
            if random.random() < self.dust_transparency_val:
                modified_msg.ranges[index_value] = modified_msg.ranges[index_value]  # The LiDAR dist remains the same
            else:
                modified_msg.ranges[index_value] = self.place_scan_in_zone(modified_msg, index_value)  # Dust detected


    def place_scan_in_zone(self, modified_msg : LaserScan, index_value):
        # This function determines the correct distance to give the modified scan so the distance is in the dust zone.
        # We try random distances and if the scan is in the dust zone, we keep it.
        scan_angle = (index_value + self.lidar_list_offset) * 2*np.pi/(len(modified_msg.ranges)-1)

        potential_scan_point = PointStamped()
        potential_scan_point.header.frame_id = 'base_link'
        potential_scan_point.header.stamp = self.get_clock().now().to_msg()
        potential_scan_point.point.x = 0.0
        potential_scan_point.point.y = 0.0
        potential_scan_point.point.z = 0.0

        self.max_zone_distance = self.calc_max_dist_to_zone()
        
        for _ in range(5):
            # We try 10 different random distances for the new points t fit them in the dust zone
            random_distance = random.uniform(self.dist_to_zone, self.max_zone_distance)
            potential_scan_point.point.x = random_distance * np.cos(scan_angle)
            potential_scan_point.point.y = random_distance * np.sin(scan_angle)

            # Lookup the transform from 'map' to 'base_link'
            try:
                transform2 = self.tf_buffer2.lookup_transform('map', 'base_link', rclpy.time.Time())
            except Exception as e:
                pass

            # Convert point to map frame
            try:
                potential_scan_point_map = do_transform_point(potential_scan_point, transform2)
                
                if self.point_in_zone(potential_scan_point_map) == True and modified_msg.ranges[index_value] > random_distance and modified_msg.ranges[index_value] != np.inf:
                    return random_distance
            except Exception as e:
                pass

        return modified_msg.ranges[index_value]  # If no point appears in the dust zone, keep initial value


    def calc_max_dist_to_zone(self):
        # This function returns the shortest distance between the zone and the robot in the robot tf
        dist_to_zone_list = []
        distance_1 = self.calc_dist_to_segment(self.transformed_zone_points_list[0], self.transformed_zone_points_list[1])
        dist_to_zone_list.append(distance_1)
        distance_2 = self.calc_dist_to_segment(self.transformed_zone_points_list[1], self.transformed_zone_points_list[2])
        dist_to_zone_list.append(distance_2)
        distance_3 = self.calc_dist_to_segment(self.transformed_zone_points_list[2], self.transformed_zone_points_list[3])
        dist_to_zone_list.append(distance_3)
        distance_4 = self.calc_dist_to_segment(self.transformed_zone_points_list[3], self.transformed_zone_points_list[0])
        dist_to_zone_list.append(distance_4)
        return np.max(dist_to_zone_list)    


    def calc_dist_to_zone(self):
        # This function returns the shortest distance between the zone and the robot in the robot tf
        dist_to_zone_list = []
        distance_1 = self.calc_dist_to_segment(self.transformed_zone_points_list[0], self.transformed_zone_points_list[1])
        dist_to_zone_list.append(distance_1)
        distance_2 = self.calc_dist_to_segment(self.transformed_zone_points_list[1], self.transformed_zone_points_list[2])
        dist_to_zone_list.append(distance_2)
        distance_3 = self.calc_dist_to_segment(self.transformed_zone_points_list[2], self.transformed_zone_points_list[3])
        dist_to_zone_list.append(distance_3)
        distance_4 = self.calc_dist_to_segment(self.transformed_zone_points_list[3], self.transformed_zone_points_list[0])
        dist_to_zone_list.append(distance_4)
        self.dist_to_zone = np.min(dist_to_zone_list)


    def calc_dist_to_segment(self, p1, p2):
        robot_coordinate = [0, 0]  # In the robot's tf, the position of the robot is [0, 0]
        A = robot_coordinate[0] - p1.point.x
        B = robot_coordinate[1] - p1.point.y
        C = p2.point.x - p1.point.x
        D = p2.point.y - p1.point.y

        dot = A*C + B*D
        len_sq = C**2 + D**2
        param = -1
        if len_sq != 0:  # in case of 0 length line
            param = dot/len_sq

        if param < 0:
            xx = p1.point.x
            yy = p1.point.y

        elif param > 1:
            xx = p2.point.x
            yy = p2.point.y

        else:
            xx = p1.point.x + param * C
            yy = p1.point.y + param * D

        dx = robot_coordinate[0] - xx
        dy = robot_coordinate[1] - yy
        return np.sqrt(dx**2 + dy**2)


    def calc_angle_to_zone(self):
        # This function returns the widest range for the dust zone. If the angle step up (0 = 360) is in the direction of the zone, errors might form.
        robot_coordinate = [0, 0]  # In the robot's tf, the position of the robot is [0, 0]
        obstacle_angle_list = []
        for transformed_point in self.transformed_zone_points_list:
            angle = np.arctan2(transformed_point.point.y-robot_coordinate[1], transformed_point.point.x-robot_coordinate[0]) + np.pi # Returns the angle between 0 and 2*pi
            obstacle_angle_list.append(angle)  # Returns the angle in degrees
        
        self.angle_to_zone = [np.min(obstacle_angle_list), np.max(obstacle_angle_list)]


    def point_in_zone(self, sim_point : PointStamped):
        if self.zone_points_list[2].point.x < sim_point.point.x and sim_point.point.x < self.zone_points_list[1].point.x and self.zone_points_list[1].point.y < sim_point.point.y and sim_point.point.y < self.zone_points_list[0].point.y:
            return True
    

    def lidar_callback(self, msg : LaserScan):
        # This function is called when the robot gets LiDAR measurements
        self.transform_points()  # We transform the zone corners to the robot frame

        try:
            # Calculate the shortest distance from the robot to the dust zone
            self.calc_dist_to_zone()
            # Calculate the angle range of the dust zone to the robot
            self.calc_angle_to_zone()
            
            #self.get_logger().info(f"Distance to dust zone: {self.dist_to_zone}")
            #self.get_logger().info(f"Angle to dust zone: {self.angle_to_zone}")
        except Exception as e:
            self.get_logger().error(f"Could not transform point: {str(e)}")

        modified_msg = LaserScan()
        modified_msg.header = msg.header
        modified_msg.angle_min = msg.angle_min
        modified_msg.angle_max = msg.angle_max
        modified_msg.angle_increment = msg.angle_increment
        modified_msg.time_increment = msg.time_increment
        modified_msg.scan_time = msg.scan_time
        modified_msg.range_min = msg.range_min
        modified_msg.range_max = msg.range_max
        modified_msg.intensities = msg.intensities
        modified_msg.ranges = msg.ranges

        if self.dust_transparency_val != 1.0:
            # Change LiDAR ranges and intensities for the new modified_msg
            try:
                self.lidar_modifications(modified_msg)   
            except Exception as e:
                self.get_logger().error(f"Could not transform point: {str(e)}")

        self.image_publisher.publish(modified_msg)

    
    def create_marker(self, point_stamped, frame_id, marker_id):
        # This function creates markers to be used to visualize the obstacle points in rviz2
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
    
    def timer_callback(self):
        self.get_logger().info("lidar_dust_noise: Active")
    


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDustCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

