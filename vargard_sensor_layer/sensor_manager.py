
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
        video_devices = sorted(glob.glob('/dev/video*'))
        for dev in video_devices:
            try:
                # Extract device index
                dev_idx = int(dev.replace('/dev/video', ''))

                # Test if device is accessible
                cap = cv2.VideoCapture(dev_idx)
                if cap.isOpened():
                    # Check if we can actually read frames
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        # Detect FLIR thermal cameras by device info
                        sensor_type = 'usb_camera'
                        try:
                            info = subprocess.check_output(
                                ['v4l2-ctl', '-d', dev, '--info'],
                                stderr=subprocess.DEVNULL,
                                timeout=5
                            ).decode()
                            if 'FLIR' in info.upper():
                                sensor_type = 'flir_thermal'
                        except (
                            subprocess.TimeoutExpired,
                            subprocess.CalledProcessError,
                            FileNotFoundError):
                            # v4l2-ctl not available or failed, stick with usb_camera
                            pass
                        detected.append((sensor_type, dev_idx))
                cap.release()
            except (ValueError, cv2.error, Exception):
                # Skip invalid devices
                pass

        # Detect CSI camera (Jetson specific)
        try:
            # Try to create CSI camera to test availability
            test_csi = CsiCamera()
            # If successful, add to detected list
            detected.append(('csi_camera', None))
            # Clean up test instance
            try:
                test_csi.close()
            except Exception:
                pass
        except Exception:
            # CSI camera not available
            pass

        # Detect radar serial ports with basic validation
        serial_ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        for port in serial_ports:
            try:
                # Basic check if port exists and is accessible
                if os.path.exists(port) and os.access(port, os.R_OK | os.W_OK):
                    detected.append(('radar', port))
            except Exception:
                pass

        if detected:
            # Instantiate detected sensors
            for sensor_type, conn in detected:
                try:
                    sensor = None
                    if sensor_type == 'usb_camera':
                        sensor = UsbCamera(conn)  # conn is already device index
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

                    if sensor:
                        # No calibration or extrinsics for auto-detected sensors
                        setattr(sensor, 'calibration_file', None)
                        setattr(sensor, 'parent_frame', None)
                        setattr(sensor, 'extrinsics', None)
                        self.sensors.append(sensor)

                except Exception as e:
                    print(f'Failed to initialize {sensor_type} ({conn}): {e}')
                    # Continue with other sensors even if one fails
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
                            sensor = RadarSensor(
                                entry.get('port'),
                                entry.get('baudrate',
                                115200))
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
