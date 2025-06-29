import types
import sys
sys.modules.setdefault('cv2', types.SimpleNamespace(VideoCapture=lambda *a, **k: types.SimpleNamespace(isOpened=lambda: False, release=lambda: None)))
sys.modules.setdefault('serial', types.SimpleNamespace())
from vargard_sensor_layer.sensor_manager import SensorManager

class DummySensor:
    def __init__(self, *a, **k):
        self.sensor_id = 'dummy'
        self.sensor_type = 'dummy'


def test_manager_loads_config(tmp_path, monkeypatch):
    cfg = """
sensors:
  - type: usb_camera
    device_index: 0
  - type: radar
    port: /dev/ttyUSB0
"""
    yaml_path = tmp_path / "sensors.yaml"
    yaml_path.write_text(cfg)

    monkeypatch.setattr('vargard_sensor_layer.sensor_manager.UsbCamera', lambda idx: DummySensor())
    monkeypatch.setattr('vargard_sensor_layer.sensor_manager.RadarSensor', lambda port, *a: DummySensor())
    sm = SensorManager(str(yaml_path))
    sensors = sm.get_sensors()
    assert len(sensors) == 2

