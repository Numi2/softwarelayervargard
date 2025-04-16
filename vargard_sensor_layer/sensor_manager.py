
"""
SensorManager: auto-detects sensors or loads from YAML config.
"""

import os
import glob
import yaml
import cv2
import subprocess
from .usb_camera import UsbCamera
from .csi_camera import CsiCamera
from .flir_sensor import FlirSensor
from .ip_camera import IPCamera
from .radar_sensor import RadarSensor

class SensorManager:
    def __init__(self, config_file: str = None):
        self.config_file = config_file
        self.sensors = []
        self._detect_sensors()

    def _detect_sensors(self):
        # Reset sensor list before (re)detecting
        self.sensors = []
        detected = []
        # Detect USB cameras (/dev/video*)
        video_devices = glob.glob('/dev/video*')
        for dev in video_devices:
            try:
                cap = cv2.VideoCapture(dev)
                if cap.isOpened():
                    # detect FLIR thermal cameras by device info
                    sensor_type = 'usb_camera'
                    try:
                        info = subprocess.check_output(['v4l2-ctl', '-d', dev, '--info'], stderr=subprocess.DEVNULL).decode()
                        if 'FLIR' in info:
                            sensor_type = 'flir_thermal'
                    except Exception:
                        pass
                    detected.append((sensor_type, dev))
                cap.release()
            except Exception:
                pass

        # Detect CSI camera
        try:
            csi = CsiCamera()
            detected.append(('csi_camera', None))
        except Exception:
            pass

        # Detect radar serial ports
        serial_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        for port in serial_ports:
            detected.append(('radar', port))

        if detected:
            # Instantiate detected sensors
            for sensor_type, conn in detected:
                try:
                    if sensor_type == 'usb_camera':
                        idx = int(conn.replace('/dev/video', ''))
                        sensor = UsbCamera(idx)
                    elif sensor_type == 'csi_camera':
                        sensor = CsiCamera()
                    elif sensor_type == 'flir_thermal':
                        sensor = FlirSensor(conn)
                    elif sensor_type == 'ip_camera':
                        sensor = IPCamera(conn)
                    elif sensor_type == 'radar':
                        sensor = RadarSensor(conn)
                    else:
                        continue
                    # No calibration or extrinsics for auto-detected sensors
                    setattr(sensor, 'calibration_file', None)
                    setattr(sensor, 'parent_frame', None)
                    setattr(sensor, 'extrinsics', None)
                    self.sensors.append(sensor)
                except Exception as e:
                    print(f'Failed to init {sensor_type} ({conn}): {e}')
        else:
            # Fallback to YAML config
            if self.config_file and os.path.exists(self.config_file):
                with open(self.config_file) as f:
                    cfg = yaml.safe_load(f)
                for entry in cfg.get('sensors', []):
                    stype = entry.get('type')
                    try:
                        if stype == 'usb_camera':
                            sensor = UsbCamera(entry.get('device_index', 0))
                        elif stype == 'csi_camera':
                            params = entry.get('params', {})
                            sensor = CsiCamera(**params)
                        elif stype == 'flir_thermal':
                            sensor = FlirSensor(entry.get('device_path'))
                        elif stype == 'ip_camera':
                            sensor = IPCamera(entry.get('rtsp_url'))
                        elif stype == 'radar':
                            sensor = RadarSensor(entry.get('port'), entry.get('baudrate', 115200))
                        else:
                            continue
                        # Attach calibration and extrinsics if provided
                        sensor.calibration_file = entry.get('calibration_file')
                        sensor.parent_frame = entry.get('parent_frame')
                        sensor.extrinsics = entry.get('extrinsics')
                        self.sensors.append(sensor)
                    except Exception as e:
                        print(f'Failed to init from config: {e}')
            else:
                raise RuntimeError('No sensors detected and no valid config provided')

    def get_sensors(self):
        return self.sensors
    def refresh(self):
        """
        Re-detect sensors and update the internal list.

        Returns:
            list: current list of sensor instances
        """
        self._detect_sensors()
        return self.sensors