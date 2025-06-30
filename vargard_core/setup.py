from setuptools import setup

package_name = 'vargard_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/Alert.msg']),
        ('share/' + package_name + '/launch', ['launch/demo_inference.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'torch',
        'opencv-python',
        'pyyaml',
        'requests',
        'psutil',
        'paho-mqtt',
        'click',
        'ultralytics',
        'camera-info-manager',
    ],
    zip_safe=True,
    maintainer='Vargard Team',
    maintainer_email='maintainer@example.com',
    description='Core inference & event management for Vargard AI Brain',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'inference_node = vargard_core.inference_node:main',
            'event_manager = vargard_core.event_manager:main',
            'telemetry_node = vargard_core.telemetry_node:main',
            'vargardctl = vargard_core.cli:cli',
        ],
        # Register available inference plugins
        'vargard.inference_plugins': [
            'yolov8 = vargard_core.plugins.yolov8:YOLOv8Plugin',
            'yolov5 = vargard_core.plugins.yolov5:YOLOv5Plugin'
        ],
    },
)