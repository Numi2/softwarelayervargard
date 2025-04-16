"""
ROS2 node for edge-to-cloud telemetry and OTA updates via MQTT.
Collects system metrics (CPU, memory, temp, network, GPU) and publishes to MQTT.
Subscribes to a config topic for OTA and remote config pushes.
"""
import json
import time
import subprocess
import rclpy
from rclpy.node import Node
import psutil
import paho.mqtt.client as mqtt

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')
        # MQTT parameters
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 8883)
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('tls_ca_cert', '')
        self.declare_parameter('tls_certfile', '')
        self.declare_parameter('tls_keyfile', '')
        self.declare_parameter('telemetry_topic', 'vargard/telemetry')
        self.declare_parameter('config_topic', 'vargard/config')
        self.declare_parameter('publish_interval', 5.0)

        self.broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.keepalive = self.get_parameter('mqtt_keepalive').get_parameter_value().integer_value
        self.ca_cert = self.get_parameter('tls_ca_cert').get_parameter_value().string_value
        self.certfile = self.get_parameter('tls_certfile').get_parameter_value().string_value
        self.keyfile = self.get_parameter('tls_keyfile').get_parameter_value().string_value
        self.telemetry_topic = self.get_parameter('telemetry_topic').get_parameter_value().string_value
        self.config_topic = self.get_parameter('config_topic').get_parameter_value().string_value
        self.interval = self.get_parameter('publish_interval').get_parameter_value().double_value

        # MQTT client setup
        self.client = mqtt.Client()
        if self.ca_cert:
            self.client.tls_set(ca_certs=self.ca_cert,
                                certfile=self.certfile or None,
                                keyfile=self.keyfile or None)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker, self.port, self.keepalive)
        self.client.loop_start()
        # Subscribe for OTA/config messages
        self.client.subscribe(self.config_topic)

        # Track network for delta computation
        self.last_net = psutil.net_io_counters()
        # Timer to periodically publish telemetry
        self.create_timer(self.interval, self.publish_telemetry)

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker {self.broker}:{self.port} (rc={rc})")

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            data = json.loads(payload)
        except Exception as e:
            self.get_logger().error(f"Failed to parse config message: {e}")
            return
        self.get_logger().info(f"Config message on {msg.topic}: {data}")
        # Remote config: update sensors.yaml
        if 'sensors.yaml' in data:
            new_yaml = data['sensors.yaml']
            try:
                with open('sensors.yaml', 'w') as f:
                    f.write(new_yaml)
                self.get_logger().info("Updated sensors.yaml on disk")
            except Exception as e:
                self.get_logger().error(f"Failed to write sensors.yaml: {e}")
        # OTA update: docker image swap
        if 'update_image' in data:
            image = data['update_image']
            self.get_logger().info(f"OTA update requested: {image}")
            try:
                subprocess.Popen(['/bin/bash', 'supervisor.sh', image])
            except Exception as e:
                self.get_logger().error(f"Failed to trigger OTA update: {e}")

    def publish_telemetry(self):
        # CPU and memory
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent
        # Temperatures
        temps = psutil.sensors_temperatures() or {}
        # Network delta
        net = psutil.net_io_counters()
        sent = net.bytes_sent - self.last_net.bytes_sent
        recv = net.bytes_recv - self.last_net.bytes_recv
        self.last_net = net
        # GPU metrics via nvidia-smi fallback
        gpu = {}
        try:
            out = subprocess.check_output([
                'nvidia-smi',
                '--query-gpu=utilization.gpu,temperature.gpu',
                '--format=csv,noheader,nounits'
            ])
            util, temp = out.decode().strip().split(',')
            gpu = {'utilization': float(util), 'temperature': float(temp)}
        except Exception:
            gpu = {}
        # Assemble payload
        payload = {
            'timestamp': time.time(),
            'cpu_percent': cpu,
            'memory_percent': mem,
            'temperatures': {k: [t.current for t in v] for k, v in temps.items()},
            'network_bytes_sent': sent,
            'network_bytes_recv': recv,
            'gpu': gpu
        }
        # Publish
        try:
            self.client.publish(self.telemetry_topic, json.dumps(payload))
            self.get_logger().debug(f"Published telemetry to {self.telemetry_topic}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish telemetry: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Cleanup
    node.client.loop_stop()
    node.client.disconnect()
    node.destroy_node()
    rclpy.shutdown()