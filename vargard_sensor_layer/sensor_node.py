
"""
ROS2 node to publish sensor data on topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from .sensor_manager import SensorManager

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_layer_node')
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.manager = SensorManager(config_file)
        self.bridge = CvBridge()
        self.publishers = {}

        for sensor in self.manager.get_sensors():
            topic = f'/sensor/{sensor.sensor_id}'
            if sensor.sensor_type in ['usb_camera', 'csi_camera', 'ip_camera', 'flir_thermal']:
                self.publishers[sensor.sensor_id] = self.create_publisher(Image, topic, 10)
            elif sensor.sensor_type == 'radar':
                self.publishers[sensor.sensor_id] = self.create_publisher(String, topic, 10)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        for sensor in self.manager.get_sensors():
            try:
                data, meta = sensor.get_frame()
                if sensor.sensor_type in ['usb_camera', 'csi_camera', 'ip_camera', 'flir_thermal']:
                    msg = self.bridge.cv2_to_imgmsg(data, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = sensor.sensor_id
                    self.publishers[sensor.sensor_id].publish(msg)
                elif sensor.sensor_type == 'radar':
                    msg = String()
                    msg.data = data.decode('utf-8', errors='ignore') if isinstance(data, (bytes, bytearray)) else str(data)
                    self.publishers[sensor.sensor_id].publish(msg)
            except Exception as e:
                self.get_logger().warn(f'Error reading sensor {sensor.sensor_id}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()