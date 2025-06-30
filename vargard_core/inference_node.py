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
        try:
            from importlib.metadata import entry_points
            eps = entry_points(group='vargard.inference_plugins')
        except ImportError:
            # Fallback for older Python versions
            import pkg_resources
            eps = pkg_resources.iter_entry_points('vargard.inference_plugins')
        
        for ep in eps:
            try:
                cls = ep.load()
                plugin = cls()
                
                # Load and validate parameters for plugin
                param_name = f'plugin.{ep.name}.config'
                self.declare_parameter(param_name, '{}')
                cfg = self.get_parameter(param_name).get_parameter_value().string_value
                params = json.loads(cfg) if isinstance(cfg, str) and cfg else {}
                
                # Validate configuration if plugin supports it
                if hasattr(plugin, 'validate_config') and not plugin.validate_config(params):
                    self.get_logger().error(f'Invalid configuration for plugin {ep.name}')
                    continue
                
                # Initialize plugin
                plugin.initialize(params)
                
                # Check if initialization was successful
                if hasattr(plugin, 'status') and plugin.status.value == 'ready':
                    self.plugins[ep.name] = plugin
                    self.get_logger().info(f'Successfully loaded plugin: {ep.name} v{getattr(plugin, "version", "unknown")}')
                    
                    # Log plugin info
                    if hasattr(plugin, 'get_info'):
                        info = plugin.get_info()
                        self.get_logger().info(f'Plugin {ep.name}: {info.get("description", "No description")}')
                else:
                    self.get_logger().error(f'Plugin {ep.name} failed to initialize properly')
                    
            except Exception as e:
                self.get_logger().error(f'Failed to load plugin {ep.name}: {e}')
        # Data storage
        self.bridge = CvBridge()
        self.camera_info = {}
        # Subscriptions to sensor topics discovered at runtime
        self.subscriptions = {}
        self._scan_topics()  # subscribe to existing topics at startup
        # periodic scan for newly created/removed topics
        self.create_timer(5.0, self._scan_topics)
        # Publishers for plugin outputs
        self.publishers = {}
        for name in self.plugins:
            qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            topic = f'/vargard/events/{name}'
            self.publishers[name] = self.create_publisher(String, topic, qos)

    def _scan_topics(self):
        """Subscribe/unsubscribe to sensor topics dynamically."""
        current_topics = {t for t, _ in self.get_topic_names_and_types()}
        # Subscribe to new topics
        for topic, types in self.get_topic_names_and_types():
            if topic not in self.subscriptions:
                subs = []
                if 'sensor_msgs/msg/CameraInfo' in types:
                    subs.append(self.create_subscription(CameraInfo, topic, self._info_cb, 10))
                if 'sensor_msgs/msg/Image' in types:
                    subs.append(self.create_subscription(Image, topic, self._image_cb, 10))
                if subs:
                    self.subscriptions[topic] = subs
        # Remove subscriptions for disappeared topics
        for topic in list(self.subscriptions.keys()):
            if topic not in current_topics:
                for sub in self.subscriptions.pop(topic):
                    try:
                        self.destroy_subscription(sub)
                    except Exception:
                        pass

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
                # Skip disabled plugins
                if hasattr(plugin, 'status') and plugin.status.value == 'disabled':
                    continue
                
                # Use enhanced processing with timing if available
                if hasattr(plugin, 'process_with_timing'):
                    dets, inference_time = plugin.process_with_timing(img, caminfo)
                else:
                    dets = plugin.process(img, caminfo)
                    inference_time = None
                
                # Create enhanced payload
                payload = {
                    'sensor_id': msg.header.frame_id,
                    'plugin': name,
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'detections': dets,
                    'detection_count': len(dets),
                }
                
                # Add performance metrics if available
                if inference_time is not None:
                    payload['inference_time'] = inference_time
                
                if hasattr(plugin, 'get_info'):
                    info = plugin.get_info()
                    payload['plugin_stats'] = {
                        'inference_count': info.get('inference_count', 0),
                        'average_inference_time': info.get('average_inference_time', 0),
                        'error_count': info.get('error_count', 0)
                    }
                
                # Publish results
                s = String()
                s.data = json.dumps(payload)
                self.publishers[name].publish(s)
                
            except Exception as e:
                self.get_logger().warn(f'Plugin {name} failed: {e}')
                # Update plugin error count if supported
                if hasattr(plugin, 'error_count'):
                    plugin.error_count += 1

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