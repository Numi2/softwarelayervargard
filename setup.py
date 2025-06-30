
import os
from setuptools import setup

package_name = 'vargard_sensor_layer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['vargard_sensor_layer/msg/SensorFrame.msg']),
    ],
    install_requires=['setuptools', 'opencv-python', 'pyserial'],
    zip_safe=True,
    maintainer='Vargard Team',
    maintainer_email='maintainer@example.com',
    description='Modular sensor input layer for Vargard Core',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = vargard_sensor_layer.sensor_node:main'
        ],
    },
)