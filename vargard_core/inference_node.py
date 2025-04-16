"""
ROS2 node that loads inference plugins and processes incoming image streams.
"""
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import pkg_resources
import os
from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class InferenceNode(Node):
    def __init__(self):
        super().__init__('inference_node')
        # Diagnostics support
        self.declare_parameter('enable_diagnostics', False)
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').get_parameter_value().bool_value
        self.updater = Updater(self)
        self.updater.setHardwareID(self.get_name())
        self.updater.add('Node Status', self._diag_status)
        if self.enable_diagnostics:
            qos_diag = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
            self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile=qos_diag)
        self.create_timer(1.0, self.updater.update)
        # Load plugins via entry points
        self.plugins = {}
        for ep in pkg_resources.iter_entry_points('vargard.inference_plugins'):
            try:
                cls = ep.load()
                plugin = cls()
                # load params for plugin
                param_name = f'plugin.{ep.name}.config'
                self.declare_parameter(param_name, {})
                cfg = self.get_parameter(param_name).get_parameter_value().string_value
                params = json.loads(cfg) if isinstance(cfg, str) and cfg else {}
                plugin.initialize(params)
                self.plugins[ep.name] = plugin
                self.get_logger().info(f'Loaded plugin: {ep.name}')
            except Exception as e:
                self.get_logger().error(f'Failed to load plugin {ep.name}: {e}')
        # Data storage
        self.bridge = CvBridge()
        self.camera_info = {}
        # Subscriptions to existing topics
        topics = self.get_topic_names_and_types()
        for topic, types in topics:
            if 'sensor_msgs/msg/CameraInfo' in types:
                self.create_subscription(CameraInfo, topic, self._info_cb, 10)
            if 'sensor_msgs/msg/Image' in types:
                self.create_subscription(Image, topic, self._image_cb, 10)
        # Publishers for plugin outputs
        self.publishers = {}
        for name in self.plugins:
            qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            topic = f'/vargard/events/{name}'
            self.publishers[name] = self.create_publisher(String, topic, qos)

    def _info_cb(self, msg: CameraInfo):
        # cache latest camera_info by frame_id
        self.camera_info[msg.header.frame_id] = msg

    def _image_cb(self, msg: Image):
        # on each image, run all plugins
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        caminfo = self.camera_info.get(msg.header.frame_id)
        for name, plugin in self.plugins.items():
            try:
                dets = plugin.process(img, caminfo)
                payload = {
                    'sensor_id': msg.header.frame_id,
                    'plugin': name,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'detections': dets,
                }
                s = String()
                s.data = json.dumps(payload)
                self.publishers[name].publish(s)
            except Exception as e:
                self.get_logger().warn(f'Plugin {name} failed: {e}')

    def _diag_status(self, stat):
        """Basic diagnostic task for node heartbeat."""
        stat.summary(DiagnosticStatus.OK, 'Node running')
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()