import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray


class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a sample point in the 'map' frame
        self.point_in_map = PointStamped()
        self.point_in_map.header.frame_id = 'map'
        self.point_in_map.header.stamp = self.get_clock().now().to_msg()
        self.point_in_map.point.x = 1.0
        self.point_in_map.point.y = 2.0
        self.point_in_map.point.z = 0.0

        # Schedule the transformation after 2 seconds to ensure tf2 has time to receive transforms
        self.timer = self.create_timer(2.0, self.transform_point)

        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)


    def transform_point(self):
        try:
            # Lookup the transform from 'map' to 'base_link'
            transform = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
            # Transform the point
            transformed_point = do_transform_point(self.point_in_map, transform)
            self.get_logger().info(f"Transformed point: {transformed_point.point}")
        except Exception as e:
            self.get_logger().error(f"Could not transform point: {str(e)}")

        try:
            map_marker = self.create_marker(self.point_in_map, 'map', 0)
            base_link_marker = self.create_marker(transformed_point, 'base_link', 1)
            self.marker_publisher.publish(map_marker)
            self.marker_publisher.publish(base_link_marker)
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
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
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
