import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Configurarea și publicarea transformării statice
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"  # Cadrul de referință de bază
        static_transform.child_frame_id = "lidar_frame"  # Cadrul de referință pentru lidar

        # Setăm translatarea și rotația
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # Identitatea quaternionului

        # Publicăm transformarea statică
        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Transformare statică publicată între 'base_link' și 'lidar_frame'.")

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
