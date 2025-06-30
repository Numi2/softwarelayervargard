
"""
ROS2 node to publish sensor data on topics, with camera_info, TF, QoS, hot‑plug & diagnostics support.
"""
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from cv_bridge import CvBridge
try:
    from camera_info_manager import CameraInfoManager
    HAS_CAMERA_INFO_MANAGER = True
except ImportError:
    HAS_CAMERA_INFO_MANAGER = False
    # Fallback implementation for when camera_info_manager is not available
    class CameraInfoManager:
        def __init__(self, node, camera_name, url=''):
            self.node = node
            self.camera_name = camera_name
            self.url = url
            self._camera_info = None
            
        def getCameraInfo(self):
            from sensor_msgs.msg import CameraInfo
            if self._camera_info is None:
                self._camera_info = CameraInfo()
                # Set default values
                self._camera_info.width = 640
                self._camera_info.height = 480
            return self._camera_info
            
import tf2_ros
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as ROS2Time
from .sensor_manager import SensorManager

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_layer_node')
        # Parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('hotplug_interval', 5.0)
        self.declare_parameter('calibration_folder', '')
        self.declare_parameter('use_hardware_timestamp', False)
        self.declare_parameter('enable_diagnostics', False)
        # Get parameter values
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        hotplug_interval = self.get_parameter('hotplug_interval').get_parameter_value().double_value
        self.calibration_folder = self.get_parameter('calibration_folder').get_parameter_value().string_value
        self.use_hw_ts = self.get_parameter('use_hardware_timestamp').get_parameter_value().bool_value
        self.enable_diagnostics = self.get_parameter('enable_diagnostics').get_parameter_value().bool_value
        # Sensor manager
        self.manager = SensorManager(config_file)
        # Utilities
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # State holders
        self.publishers = {}
        self.ci_managers = {}
        self.extrinsics = {}
        self.sensor_objects = {}
        # Diagnostics: ROS diagnostic_updater
        from diagnostic_updater import Updater
        # Create updater
        self.updater = Updater(self)
        self.updater.setHardwareID(self.get_name())
        self.updater.add('Node Status', self._diag_status)
        # Diagnostic array publisher
        if self.enable_diagnostics:
            self.last_data_time = {}
            qos_diag = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
            self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile=qos_diag)
        # Timer for updater
        self.create_timer(1.0, self.updater.update)
        # Initial sensor setup
        for sensor in self.manager.get_sensors():
            self._add_sensor(sensor)
        # Timers
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.hotplug_timer = self.create_timer(hotplug_interval, self.hotplug_callback)
        if self.enable_diagnostics:
            self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

    def timer_callback(self):
        # Poll each sensor and publish data
        for sid, sensor in list(self.sensor_objects.items()):
            try:
                data, meta = sensor.get_frame()
                # Image sensors
                if sensor.sensor_type in ['usb_camera', 'csi_camera', 'ip_camera', 'flir_thermal']:
                    img_msg = self.bridge.cv2_to_imgmsg(data, encoding='bgr8')
                    # Timestamp: hardware vs ROS time
                    if self.use_hw_ts and meta and 'timestamp' in meta:
                        t = meta['timestamp']
                        sec = int(t)
                        nanosec = int((t - sec) * 1e9)
                        img_msg.header.stamp = ROS2Time(sec=sec, nanosec=nanosec)
                    else:
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = sid
                    # Publish image
                    self.publishers[sid]['image'].publish(img_msg)
                    # Publish camera info
                    ci = self.ci_managers[sid].getCameraInfo()
                    ci.header = img_msg.header
                    self.publishers[sid]['info'].publish(ci)
                    # Broadcast TF if extrinsics available
                    if sid in self.extrinsics:
                        extr = self.extrinsics[sid]
                        tf_msg = TransformStamped()
                        tf_msg.header.stamp = img_msg.header.stamp
                        tf_msg.header.frame_id = extr.get('parent_frame', '') or ''
                        tf_msg.child_frame_id = sid
                        tvec = extr.get('translation', [0.0, 0.0, 0.0])
                        rquat = extr.get('rotation', [0.0, 0.0, 0.0, 1.0])
                        tf_msg.transform.translation.x = tvec[0]
                        tf_msg.transform.translation.y = tvec[1]
                        tf_msg.transform.translation.z = tvec[2]
                        tf_msg.transform.rotation.x = rquat[0]
                        tf_msg.transform.rotation.y = rquat[1]
                        tf_msg.transform.rotation.z = rquat[2]
                        tf_msg.transform.rotation.w = rquat[3]
                        self.tf_broadcaster.sendTransform(tf_msg)
                # Radar sensor
                elif sensor.sensor_type == 'radar':
                    msg = String()
                    msg.data = data.decode('utf-8', errors='ignore') if isinstance(data, (bytes, bytearray)) else str(data)
                    self.publishers[sid]['radar'].publish(msg)
                # Update diagnostics timestamp
                if self.enable_diagnostics:
                    self.last_data_time[sid] = time.time()
                    
            except Exception as e:
                self.get_logger().warn(f'Error reading sensor {sid}: {e}')
                # Check if sensor needs reconnection
                if hasattr(sensor, 'status') and sensor.status.value == 'failed':
                    self.get_logger().info(f'Attempting to reconnect sensor {sid}')
                    if hasattr(sensor, 'attempt_reconnect') and sensor.attempt_reconnect():
                        self.get_logger().info(f'Successfully reconnected sensor {sid}')
                    else:
                        self.get_logger().warn(f'Failed to reconnect sensor {sid}')

    def _get_qos(self, sensor, info=False):
        # QoS tuning per sensor type
        if sensor.sensor_type in ['usb_camera', 'csi_camera', 'ip_camera', 'flir_thermal']:
            # Best effort for image; reliable for camera_info
            if info:
                return QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
            return QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        elif sensor.sensor_type == 'radar':
            return QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        return QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)

    def _add_sensor(self, sensor):
        sid = sensor.sensor_id
        self.sensor_objects[sid] = sensor
        topic = f'/sensor/{sid}'
        # Image sensor
        if sensor.sensor_type in ['usb_camera', 'csi_camera', 'ip_camera', 'flir_thermal']:
            qos_img = self._get_qos(sensor, info=False)
            qos_info = self._get_qos(sensor, info=True)
            img_pub = self.create_publisher(Image, topic, qos_profile=qos_img)
            info_pub = self.create_publisher(CameraInfo, topic + '/camera_info', qos_profile=qos_info)
            self.publishers[sid] = {'image': img_pub, 'info': info_pub}
            # Calibration loader
            calib_url = ''
            cf = getattr(sensor, 'calibration_file', None)
            if cf:
                if not cf.startswith('file://'):
                    path = cf if os.path.isabs(cf) else os.path.join(self.calibration_folder, cf)
                    calib_url = 'file://' + path
                else:
                    calib_url = cf
            ci_mgr = CameraInfoManager(self, sid, url=calib_url)
            self.ci_managers[sid] = ci_mgr
            if not HAS_CAMERA_INFO_MANAGER and calib_url:
                self.get_logger().warn(f'camera_info_manager not installed. Using fallback implementation for {sid}. Calibration file will not be loaded.')
        # Radar sensor
        elif sensor.sensor_type == 'radar':
            qos = self._get_qos(sensor)
            radar_pub = self.create_publisher(String, topic, qos_profile=qos)
            self.publishers[sid] = {'radar': radar_pub}
        # Extrinsics
        extr = getattr(sensor, 'extrinsics', None)
        if extr:
            self.extrinsics[sid] = {
                'parent_frame': getattr(sensor, 'parent_frame', ''),
                'translation': extr.get('translation', [0.0, 0.0, 0.0]),
                'rotation': extr.get('rotation', [0.0, 0.0, 0.0, 1.0])
            }
        # Initialize diagnostics timestamp
        if self.enable_diagnostics:
            self.last_data_time[sid] = 0.0

    def _remove_sensor(self, sid):
        # Clean up sensor object
        sensor = self.sensor_objects.pop(sid, None)
        if sensor:
            try:
                sensor.close()
            except Exception:
                pass
        # Destroy publishers
        pubs = self.publishers.pop(sid, {})
        for pub in pubs.values():
            try:
                self.destroy_publisher(pub)
            except Exception:
                pass
        # Remove camera info manager
        self.ci_managers.pop(sid, None)
        # Remove extrinsics
        self.extrinsics.pop(sid, None)
        # Remove diagnostics entry
        if self.enable_diagnostics:
            self.last_data_time.pop(sid, None)

    def hotplug_callback(self):
        # Check for sensor addition/removal
        try:
            new_list = self.manager.refresh()
        except Exception as e:
            self.get_logger().warn(f'Error refreshing sensors: {e}')
            return
        new_ids = {s.sensor_id for s in new_list}
        old_ids = set(self.sensor_objects.keys())
        added = new_ids - old_ids
        removed = old_ids - new_ids
        if added or removed:
            self.get_logger().info(f'Sensors changed. Added: {added}, Removed: {removed}')
            for sid in removed:
                self._remove_sensor(sid)
            for sensor in new_list:
                if sensor.sensor_id in added:
                    self._add_sensor(sensor)

    def _publish_diagnostics(self):
        # Publish diagnostic status for each sensor
        now = time.time()
        diag_arr = DiagnosticArray()
        diag_arr.header.stamp = self.get_clock().now().to_msg()
        
        for sid, sensor in self.sensor_objects.items():
            ds = DiagnosticStatus()
            ds.name = f'sensor_layer/{sid}'
            ds.hardware_id = sid
            
            # Get sensor health info if available
            if hasattr(sensor, 'get_health_info'):
                health_info = sensor.get_health_info()
                status = health_info.get('status', 'unknown')
                error_count = health_info.get('error_count', 0)
                total_frames = health_info.get('total_frames', 0)
                
                # Set diagnostic level based on sensor status
                if status == 'healthy':
                    ds.level = DiagnosticStatus.OK
                    ds.message = f'Healthy - {total_frames} frames, {error_count} errors'
                elif status == 'degraded':
                    ds.level = DiagnosticStatus.WARN
                    ds.message = f'Degraded - {error_count} recent errors'
                elif status == 'failed':
                    ds.level = DiagnosticStatus.ERROR
                    ds.message = f'Failed - {error_count} errors, attempting reconnection'
                else:
                    ds.level = DiagnosticStatus.STALE
                    ds.message = f'Unknown status: {status}'
                    
                # Add detailed health info as key-value pairs
                from diagnostic_msgs.msg import KeyValue
                for key, value in health_info.items():
                    if value is not None:
                        ds.values.append(KeyValue(key=key, value=str(value)))
            else:
                # Fallback to time-based diagnostics
                last = self.last_data_time.get(sid, 0.0)
                if now - last > 2.0:
                    ds.level = DiagnosticStatus.ERROR
                    ds.message = f'No data for >2s ({now - last:.1f}s)'
                else:
                    ds.level = DiagnosticStatus.OK
                    ds.message = 'OK'
                    
            diag_arr.status.append(ds)
        self.diag_pub.publish(diag_arr)
    def _diag_status(self, stat):
        """
        Basic diagnostic task for node heartbeat.
        """
        stat.summary(0, 'Node running')
        return stat

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Cleanup
    for sensor in list(node.sensor_objects.values()):
        try:
            sensor.close()
        except Exception:
            pass
    node.destroy_node()
    rclpy.shutdown()