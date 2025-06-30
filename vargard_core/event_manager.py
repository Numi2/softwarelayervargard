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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


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
            qos_diag = QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE)
            self.diag_pub = self.create_publisher(
                DiagnosticArray,
                '/diagnostics',
                qos_profile=qos_diag)
        self.create_timer(1.0, self.updater.update)
        # Parameters
        self.declare_parameter('rules_file', '')
        rules_file = self.get_parameter('rules_file').get_parameter_value().string_value
        # Initialize enhanced rule engine
        from .rule_engine import RuleEngine
        self.rule_engine = RuleEngine()

        # Load rules
        if rules_file and os.path.exists(rules_file):
            with open(rules_file, 'r') as f:
                cfg = yaml.safe_load(f)
            rules_config = cfg.get('rules', []) or []

            if self.rule_engine.load_rules(rules_config):
                self.get_logger(
                    ).info(f'Successfully loaded {len(rules_config)} rules from {rules_file}')
            else:
                self.get_logger(
                    ).error('Failed to load some rules - check configuration')
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
                self.subscriptions[topic] = self.create_subscription(
                    String,
                    topic,
                    self._event_cb,
                    10)
        current = set(topics.keys())
        for topic in list(self.subscriptions.keys()):
            if topic not in current:
                try:
                    self.destroy_subscription(self.subscriptions.pop(topic))
                except Exception:
                    pass

    def _diag_status(self, stat):
        """Basic diagnostic task for node heartbeat."""
        # Add rule engine statistics
        if hasattr(self, 'rule_engine'):
            rule_stats = self.rule_engine.get_rule_stats()
            total_triggers = sum(
                stats.get('triggered_count',
                0) for stats in rule_stats.values())
            total_errors = sum(
                stats.get('error_count',
                0) for stats in rule_stats.values())

            stat.summary(
                DiagnosticStatus.OK,
                f'Running - {len(rule_stats)} rules, '
                f'{total_triggers} triggers, '
                f'{total_errors} errors')

            # Add detailed rule statistics
            from diagnostic_msgs.msg import KeyValue
            stat.values.append(
                KeyValue(key='active_rules',
                value=str(len(rule_stats))))
            stat.values.append(
                KeyValue(key='total_triggers',
                value=str(total_triggers)))
            stat.values.append(
                KeyValue(key='total_errors',
                value=str(total_errors)))
        else:
            stat.summary(DiagnosticStatus.OK, 'Node running')

        return stat

    def get_rule_statistics(self):
        """Get detailed rule statistics for monitoring."""
        if hasattr(self, 'rule_engine'):
            return self.rule_engine.get_rule_stats()
        return {}

    def _event_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse event JSON: {e}')
            return

        # Use enhanced rule engine to evaluate events
        triggered_actions = self.rule_engine.evaluate_event(payload)

        # Process triggered actions
        for action_data in triggered_actions:
            rule = action_data['rule']
            event_data = action_data['event_data']

            # Find the best detection for this rule (highest confidence)
            detections = event_data.get('detections', [])
            best_detection = None

            if detections:
                # Sort by confidence and take the best one
                best_detection = max(
                    detections,
                    key=lambda d: d.get('confidence',
                    0))
            else:
                # Create a dummy detection for event-level rules
                best_detection = {
                    'class': 'event',
                    'confidence': 1.0,
                    'bbox': [0, 0, 0, 0]
                }

            # Trigger alert with enhanced context
            self._trigger_alert(
                rule,
                event_data,
                best_detection,
                action_data['timestamp'])

    def _trigger_alert(
        self,
        rule: dict,
        payload: dict,
        detection: dict,
        trigger_time: float = None):
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
        # Metadata as KeyValue array
        metadata_list = []
        metadata_list.append(KeyValue(key='rule', value=rule.get('name', '')))
        # attach detection details
        for k, v in detection.items():
            metadata_list.append(KeyValue(key=f'det_{k}', value=str(v)))
        alert.metadata = metadata_list
        # Publish alert
        self.alert_pub.publish(alert)
        self.get_logger().info(f'Alert triggered: {alert.description}')
        # External actions
        act_type = action.get('type')
        url = action.get('url')
        # Convert metadata to dict for external APIs
        meta_dict = {kv.key: kv.value for kv in metadata_list}
        # Webhook
        if act_type == 'webhook' and url:
            try:
                requests.post(url, json=meta_dict, timeout=2.0)
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
