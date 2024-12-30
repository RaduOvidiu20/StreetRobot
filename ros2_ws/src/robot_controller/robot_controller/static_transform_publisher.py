import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Transformare statică: odom -> base_link
        static_transform_odom_to_base = TransformStamped()
        static_transform_odom_to_base.header.stamp = self.get_clock().now().to_msg()
        static_transform_odom_to_base.header.frame_id = "odom"
        static_transform_odom_to_base.child_frame_id = "base_link"

        static_transform_odom_to_base.transform.translation.x = 0.0
        static_transform_odom_to_base.transform.translation.y = 0.0
        static_transform_odom_to_base.transform.translation.z = 0.0
        static_transform_odom_to_base.transform.rotation.x = 0.0
        static_transform_odom_to_base.transform.rotation.y = 0.0
        static_transform_odom_to_base.transform.rotation.z = 0.0
        static_transform_odom_to_base.transform.rotation.w = 1.0

        # Transformare statică: base_link -> lidar_frame
        static_transform_base_to_lidar = TransformStamped()
        static_transform_base_to_lidar.header.stamp = self.get_clock().now().to_msg()
        static_transform_base_to_lidar.header.frame_id = "base_link"
        static_transform_base_to_lidar.child_frame_id = "lidar_frame"

        static_transform_base_to_lidar.transform.translation.x = 0.0
        static_transform_base_to_lidar.transform.translation.y = 0.0
        static_transform_base_to_lidar.transform.translation.z = 0.0
        static_transform_base_to_lidar.transform.rotation.x = 0.0
        static_transform_base_to_lidar.transform.rotation.y = 0.0
        static_transform_base_to_lidar.transform.rotation.z = 0.0
        static_transform_base_to_lidar.transform.rotation.w = 1.0

        # Publică transformările
        self.static_broadcaster.sendTransform(static_transform_odom_to_base)
        self.static_broadcaster.sendTransform(static_transform_base_to_lidar)
        self.get_logger().info("Transformări statice publicate.")

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
