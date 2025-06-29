"""
ROS2 node to manage AI event streams and apply user-defined rules to generate alerts.
"""
import os
import json
import yaml
import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vargard_core.msg import Alert
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from diagnostic_updater import Updater
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class EventManager(Node):
    def __init__(self):
        super().__init__('event_manager')
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
        # Parameters
        self.declare_parameter('rules_file', '')
        rules_file = self.get_parameter('rules_file').get_parameter_value().string_value
        # Load rules
        self.rules = []
        if rules_file and os.path.exists(rules_file):
            with open(rules_file, 'r') as f:
                cfg = yaml.safe_load(f)
            self.rules = cfg.get('rules', []) or []
            self.get_logger().info(f'Loaded {len(self.rules)} rules from {rules_file}')
        else:
            self.get_logger().warn('No valid rules_file provided; no rules active')

        # Subscriptions: dynamically monitor /vargard/events/* topics
        self.subscriptions = {}
        self._scan_event_topics()
        self.create_timer(5.0, self._scan_event_topics)

        # Publisher for alerts
        self.alert_pub = self.create_publisher(Alert, '/vargard/alerts', 10)

    def _scan_event_topics(self):
        """Subscribe or unsubscribe based on available /vargard/events/* topics."""
        topics = dict(self.get_topic_names_and_types())
        for topic, types in topics.items():
            if topic.startswith('/vargard/events/') and topic not in self.subscriptions:
                self.subscriptions[topic] = self.create_subscription(String, topic, self._event_cb, 10)
        current = set(topics.keys())
        for topic in list(self.subscriptions.keys()):
            if topic not in current:
                try:
                    self.destroy_subscription(self.subscriptions.pop(topic))
                except Exception:
                    pass

    def _diag_status(self, stat):
        """Basic diagnostic task for node heartbeat."""
        stat.summary(DiagnosticStatus.OK, 'Node running')
        return stat

    def _event_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse event JSON: {e}')
            return
        # Evaluate rules
        for rule in self.rules:
            if payload.get('plugin') != rule.get('plugin'):
                continue
            cond = rule.get('condition', {})
            # Handle detection-based conditions
            for det in payload.get('detections', []):
                match = True
                # class equality
                if 'class' in cond and det.get('class') != cond['class']:
                    match = False
                # confidence threshold
                if 'confidence_gt' in cond and det.get('confidence', 0.0) < cond['confidence_gt']:
                    match = False
                if not match:
                    continue
                # Trigger action
                self._trigger_alert(rule, payload, det)
                # Only once per event
                break

    def _trigger_alert(self, rule: dict, payload: dict, detection: dict):
        # Prepare Alert message
        alert = Alert()
        alert.plugin = payload.get('plugin', '')
        alert.sensor_id = payload.get('sensor_id', '')
        ts = payload.get('timestamp', 0.0)
        alert.timestamp = float(ts)
        # Description with formatting
        action = rule.get('action', {})
        desc_tmpl = action.get('message', '')
        try:
            desc = desc_tmpl.format(plugin=alert.plugin,
                                     sensor_id=alert.sensor_id,
                                     value=detection.get('value', ''),
                                     rule=rule.get('name', ''))
        except Exception:
            desc = desc_tmpl
        alert.description = desc
        # Metadata map
        meta = {}
        meta['rule'] = rule.get('name', '')
        # attach detection details
        for k, v in detection.items():
            meta[f'det_{k}'] = str(v)
        alert.metadata = meta
        # Publish alert
        self.alert_pub.publish(alert)
        self.get_logger().info(f'Alert triggered: {alert.description}')
        # External actions
        act_type = action.get('type')
        url = action.get('url')
        # Webhook
        if act_type == 'webhook' and url:
            try:
                requests.post(url, json=meta, timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f'Webhook call failed: {e}')
        # Slack
        if act_type == 'slack' and url:
            try:
                from vargard_core.alerts.slack import send_slack
                send_slack(url, alert.description)
            except Exception as e:
                self.get_logger().warn(f'Slack alert failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = EventManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()