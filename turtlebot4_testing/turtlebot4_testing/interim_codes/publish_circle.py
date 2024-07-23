import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np


class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Cylinder properties
        self.c_x, self.c_y = 0.0, 0.0  # center of circle in world rf
        self.radius = 0.2  # radius of the circle
        self.n_points = 10
        self.cylinder_points = []
        self.transformed_points = []

        # Schedule the transformation after 2 seconds to ensure tf2 has time to receive transforms
        self.timer = self.create_timer(1.0, self.transform_point)

        self.marker_array_publisher = self.create_publisher(MarkerArray, 'cylinder_markers', 10)

        
    def create_cylinder(self):
        self.cylinder_points = []
        for i in range(self.n_points):
            contour_point = PointStamped()
            contour_point.header.frame_id = 'map'
            contour_point.header.stamp = self.get_clock().now().to_msg()
            contour_point.point.x = self.radius * np.cos(i/self.n_points * 2*np.pi) + self.c_x
            contour_point.point.y = self.radius * np.sin(i/self.n_points * 2*np.pi) + self.c_y
            contour_point.point.z = 0.0
            self.cylinder_points.append(contour_point)



    def transform_point(self):
        self.create_cylinder()
        self.transformed_points = []

        try:
            # Lookup the transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
            # Transform the points
            for point in self.cylinder_points:
                transformed_point = do_transform_point(point, transform)
                self.transformed_points.append(transformed_point)
                # self.get_logger().info(f"x = {transformed_point.point.x}, y = {transformed_point.point.y}")
        except Exception as e:
            self.get_logger().error(f"Could not transform point: {str(e)}")

        try:
            map_marker_array = MarkerArray()
            for id_num, map_point in enumerate(self.cylinder_points):
                map_marker = self.create_marker(map_point, 'map', id_num)
                map_marker_array.markers.append(map_marker)
            self.marker_array_publisher.publish(map_marker_array)

            base_link_marker_array = MarkerArray()
            for id_num, transformed_point in enumerate(self.transformed_points):
                base_link_marker = self.create_marker(transformed_point, 'base_link', id_num)
                base_link_marker_array.markers.append(base_link_marker)
            self.marker_array_publisher.publish(base_link_marker_array)
        except Exception as e:
            self.get_logger().error(f"Could not publish markers: {str(e)}")


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



def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()