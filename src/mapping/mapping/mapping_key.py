import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import ColorRGBA

class MarkerPublisherNode(Node):
    def __init__(self):
        super().__init__('marker_publisher_node')
        
        # Create a publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Set a timer to periodically publish the markers
        self.timer = self.create_timer(1.0, self.publish_markers)  # Publish every second

        # List of points with associated marker type (cube, sphere, cylinder, bigger cube)
        self.points = [
            (1, -1.99, -1.16),
            (2, 2.06, -1.00),
            (3, 1.87, 1.18),
            ('B', 0.16, 1.22),
            (3, -2.06, 1.07)
        ]

        self.get_logger().info('Marker publisher node started.')

    def publish_markers(self):
        # Iterate over the points and create different markers based on the first element
        for idx, point in enumerate(self.points):
            # Create a new Marker message for each point
            marker = Marker()

            # Set the frame ID and timestamp (we'll use "map" as the frame)
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            # Set the action to ADD
            marker.action = Marker.ADD

            # Set a unique ID for each marker
            marker.id = idx  # Each marker has a unique ID

            # Check the type of object (based on the first element of the point)
            if point[0] == 1:
                # Cube
                marker.type = Marker.CUBE
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
            elif point[0] == 2:
                # Sphere
                marker.type = Marker.SPHERE
                marker.scale.x = 0.08
                marker.scale.y = 0.08
                marker.scale.z = 0.08
            elif point[0] == 3:
                # Cylinder
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.08  # Radius
                marker.scale.y = 0.08  # Radius
                marker.scale.z = 0.3  # Height
            elif point[0] == 'B':
                # Bigger Cube
                marker.type = Marker.CUBE
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
            else:
                self.get_logger().warn(f"Skipping unknown marker type: {point[0]}")
                continue

            # Set the color of the marker (e.g., red)
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

            # Set the position of the marker using the pose (for non-point markers)
            marker.pose.position.x = float(point[1])
            marker.pose.position.y = float(point[2])
            marker.pose.position.z = 0.0  # Set the Z coordinate to 0

            # Publish the individual marker
            self.marker_pub.publish(marker)

        self.get_logger().info('Publishing multiple markers.')

def main(args=None):
    rclpy.init(args=args)

    marker_publisher_node = MarkerPublisherNode()

    try:
        rclpy.spin(marker_publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        marker_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
