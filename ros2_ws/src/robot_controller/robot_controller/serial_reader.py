import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import serial

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')
        self.publisher = self.create_publisher(PointCloud2, '/lidar_pointcloud', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)  # Schimbă portul dacă este necesar

        # Timer pentru a citi datele seriale periodic și a le publica imediat
        self.timer = self.create_timer(0.1, self.publish_pointcloud)

    def publish_pointcloud(self):
        # Citire date de la ESP32
        points = []
        while self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode().strip()
            if line.startswith("data:"):
                try:
                    _, coords = line.split(":")
                    x, y = map(float, coords.split(","))
                    z = 0.0  # Z este 0 pentru un nor 2D
                    
                    # Adaugă punctul la lista de puncte pentru publicare imediată
                    points.append((x, y, z))
                except ValueError:
                    self.get_logger().warning("Could not parse data from serial.")
        
        # Dacă nu sunt puncte noi, nu publicăm nimic
        if not points:
            return

        # Construim mesajul PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar_frame"

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        points_data = []
        for point in points:
            x, y, z = point
            points_data += struct.pack('fff', x, y, z)

        pointcloud_msg = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=True,
            is_bigendian=False,
            fields=fields,
            point_step=12,
            row_step=12 * len(points),
            data=points_data
        )

        # Publicăm mesajul PointCloud2
        self.publisher.publish(pointcloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
