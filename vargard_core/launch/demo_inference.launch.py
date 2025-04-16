"""
Launch file to run sensor layer, inference node, and event manager together.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to configs
    sensor_pkg = get_package_share_directory('vargard_sensor_layer')
    core_pkg = get_package_share_directory('vargard_core')
    sensors_yaml = os.path.join(sensor_pkg, 'sensors.yaml')
    rules_yaml = os.path.join(core_pkg, '..', 'rules.yaml')

    return LaunchDescription([
        Node(
            package='vargard_sensor_layer',
            executable='sensor_node',
            name='sensor_node',
            output='screen',
            parameters=[{'config_file': sensors_yaml}]
        ),
        Node(
            package='vargard_core',
            executable='inference_node',
            name='inference_node',
            output='screen'
        ),
        Node(
            package='vargard_core',
            executable='event_manager',
            name='event_manager',
            output='screen',
            parameters=[{'rules_file': rules_yaml}]
        ),
    ])