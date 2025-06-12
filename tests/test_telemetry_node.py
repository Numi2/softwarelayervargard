import types
from unittest import mock

import sys

# Create dummy rclpy and Node
class DummyNode:
    def __init__(self, name):
        pass
    def create_timer(self, *a, **k):
        pass
    def get_logger(self):
        class L:
            def info(self, *a, **k):
                pass
            def debug(self, *a, **k):
                pass
            def error(self, *a, **k):
                pass
        return L()
    def declare_parameter(self, *a, **k):
        pass
    def get_parameter(self, name):
        class P:
            def get_parameter_value(self_inner):
                class V: pass
                v = V();
                if 'mqtt_broker' in name:
                    v.string_value = 'localhost'
                elif 'config_topic' in name:
                    v.string_value = 'cfg'
                else:
                    v.string_value = ''
                if 'mqtt_port' in name:
                    v.integer_value = 1883
                else:
                    v.integer_value = 0
                v.double_value = 1.0; v.bool_value = False
                return v
        return P()

sys.modules.setdefault('rclpy', types.SimpleNamespace(init=lambda *a, **k: None, shutdown=lambda: None))
sys.modules.setdefault('rclpy.node', types.SimpleNamespace(Node=DummyNode))

from vargard_core.telemetry_node import TelemetryNode


def test_publish_telemetry(monkeypatch):
    monkeypatch.setattr('paho.mqtt.client.Client.connect', lambda self, *a, **k: None)
    monkeypatch.setattr('paho.mqtt.client.Client.loop_start', lambda self: None)
    node = TelemetryNode()

    monkeypatch.setattr(node.client, 'publish', mock.Mock())
    monkeypatch.setattr('psutil.cpu_percent', lambda: 10)
    monkeypatch.setattr('psutil.virtual_memory', lambda: types.SimpleNamespace(percent=20))
    monkeypatch.setattr('psutil.sensors_temperatures', lambda: {})
    monkeypatch.setattr('psutil.net_io_counters', lambda: types.SimpleNamespace(bytes_sent=0, bytes_recv=0))
    monkeypatch.setattr('subprocess.check_output', lambda *a, **k: b'0,0')

    node.publish_telemetry()
    assert node.client.publish.called
