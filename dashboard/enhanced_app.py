"""
Enhanced Flask backend for Vargard Dashboard with REST API and authentication.
"""

import json
import time
import threading
from collections import deque
from datetime import datetime, timedelta

from flask import Flask, jsonify, Response, request
from flask_cors import CORS
from flask_jwt_extended import (
    JWTManager,
    create_access_token,
    jwt_required,
    get_jwt_identity)
import paho.mqtt.client as mqtt

# Configuration
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
TELEMETRY_TOPIC = 'vargard/telemetry'
ALERTS_TOPIC = 'vargard/alerts'
MAX_HISTORY = 1000

app = Flask(__name__)
app.config['JWT_SECRET_KEY'] = 'vargard-secret-key-change-in-production'  # Change this!
app.config['JWT_ACCESS_TOKEN_EXPIRES'] = timedelta(hours=24)

# Extensions
CORS(app)
jwt = JWTManager(app)

# Data storage
telemetry_history = deque(maxlen=MAX_HISTORY)
alerts_history = deque(maxlen=MAX_HISTORY)
events_history = deque(maxlen=MAX_HISTORY)

# Mock data for sensors, rules, and plugins
sensors_data = [
    {
        'id': 'usb_camera_0',
        'type': 'usb_camera',
        'status': 'healthy',
        'fps': 30,
        'resolution': '1920x1080',
        'last_seen': datetime.now().isoformat()
    },
    {
        'id': 'csi_camera',
        'type': 'csi_camera',
        'status': 'healthy',
        'fps': 30,
        'resolution': '1920x1080',
        'last_seen': datetime.now().isoformat()
    }
]

rules_data = [
    {
        'id': 'person_detection',
        'name': 'Person Detection',
        'plugin': 'yolov8',
        'enabled': True,
        'conditions': {
            'class': 'person',
            'confidence_gt': 0.5
        },
        'actions': {
            'type': 'webhook',
            'url': 'http://localhost:8000/alert'
        },
        'triggered_count': 42,
        'last_triggered': datetime.now().isoformat()
    }
]

plugins_data = [
    {
        'id': 'yolov8',
        'name': 'YOLOv8',
        'version': '1.1.0',
        'status': 'ready',
        'enabled': True,
        'inference_count': 1250,
        'average_inference_time': 0.045,
        'error_count': 2
    }
]

# Mock users for authentication
users = {
    'admin': {
        'password': 'admin123',  # In production, use hashed passwords!
        'role': 'admin'
    },
    'operator': {
        'password': 'operator123',
        'role': 'operator'
    }
}

# MQTT Client Setup


def on_connect(client, userdata, flags, rc):
    print(f"Dashboard MQTT connected (rc={rc})")
    client.subscribe([(TELEMETRY_TOPIC, 0), (ALERTS_TOPIC, 0)])


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode('utf-8'))
        timestamp = datetime.now()

        if msg.topic == TELEMETRY_TOPIC:
            data['received_at'] = timestamp.isoformat()
            telemetry_history.append(data)
        elif msg.topic == ALERTS_TOPIC:
            data['received_at'] = timestamp.isoformat()
            alerts_history.append(data)
    except Exception as e:
        print(f"Error processing MQTT message: {e}")

# Start MQTT client in background thread
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    threading.Thread(target=client.loop_forever, daemon=True).start()
    print(f"Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")

# Authentication endpoints
@app.route('/api/auth/login', methods=['POST'])
def login():
    username = request.json.get('username', '')
    password = request.json.get('password', '')

    if username in users and users[username]['password'] == password:
        access_token = create_access_token(
            identity=username,
            additional_claims={'role': users[username]['role']}
        )
        return jsonify({
            'access_token': access_token,
            'user': {
                'username': username,
                'role': users[username]['role']
            }
        })

    return jsonify({'message': 'Invalid credentials'}), 401

@app.route('/api/auth/verify', methods=['GET'])
@jwt_required()
def verify_token():
    current_user = get_jwt_identity()
    return jsonify({
        'user': {
            'username': current_user,
            'role': users.get(current_user, {}).get('role', 'operator')
        }
    })

# Telemetry endpoints
@app.route('/api/telemetry')
def get_telemetry():
    n = int(request.args.get('n', len(telemetry_history)))
    return jsonify(list(telemetry_history)[-n:])

@app.route('/api/telemetry/stream')
def stream_telemetry():

    def generate():
        last_index = 0
        while True:
            while len(telemetry_history) > last_index:
                item = telemetry_history[last_index]
                yield f"data: {json.dumps(item)}\n\n"
                last_index += 1
            time.sleep(1)
    return Response(generate(), mimetype='text/event-stream')

# Alerts endpoints
@app.route('/api/alerts')
def get_alerts():
    n = int(request.args.get('n', len(alerts_history)))
    return jsonify(list(alerts_history)[-n:])

@app.route('/api/alerts/stream')
def stream_alerts():

    def generate():
        last_index = 0
        while True:
            while len(alerts_history) > last_index:
                item = alerts_history[last_index]
                yield f"data: {json.dumps(item)}\n\n"
                last_index += 1
            time.sleep(1)
    return Response(generate(), mimetype='text/event-stream')

@app.route('/api/alerts/<alert_id>/acknowledge', methods=['POST'])
@jwt_required()
def acknowledge_alert(alert_id):
    # In a real implementation, update database
    return jsonify({'message': 'Alert acknowledged', 'alert_id': alert_id})

# Sensors endpoints
@app.route('/api/sensors')
def get_sensors():
    return jsonify(sensors_data)

@app.route('/api/sensors', methods=['POST'])
@jwt_required()
def create_sensor():
    sensor_data = request.json
    # Add validation and database storage here
    sensor_data['id'] = f"sensor_{len(sensors_data)}"
    sensor_data['last_seen'] = datetime.now().isoformat()
    sensors_data.append(sensor_data)
    return jsonify(sensor_data), 201

@app.route('/api/sensors/<sensor_id>')
def get_sensor(sensor_id):
    sensor = next((s for s in sensors_data if s['id'] == sensor_id), None)
    if sensor:
        return jsonify(sensor)
    return jsonify({'message': 'Sensor not found'}), 404

@app.route('/api/sensors/<sensor_id>', methods=['PUT'])
@jwt_required()
def update_sensor(sensor_id):
    sensor = next((s for s in sensors_data if s['id'] == sensor_id), None)
    if sensor:
        sensor.update(request.json)
        return jsonify(sensor)
    return jsonify({'message': 'Sensor not found'}), 404

@app.route('/api/sensors/<sensor_id>', methods=['DELETE'])
@jwt_required()
def delete_sensor(sensor_id):
    global sensors_data
    sensors_data = [s for s in sensors_data if s['id'] != sensor_id]
    return '', 204

# Rules endpoints
@app.route('/api/rules')
def get_rules():
    return jsonify(rules_data)

@app.route('/api/rules', methods=['POST'])
@jwt_required()
def create_rule():
    rule_data = request.json
    rule_data['id'] = f"rule_{len(rules_data)}"
    rule_data['triggered_count'] = 0
    rule_data['last_triggered'] = None
    rules_data.append(rule_data)
    return jsonify(rule_data), 201

@app.route('/api/rules/<rule_id>')
def get_rule(rule_id):
    rule = next((r for r in rules_data if r['id'] == rule_id), None)
    if rule:
        return jsonify(rule)
    return jsonify({'message': 'Rule not found'}), 404

@app.route('/api/rules/<rule_id>', methods=['PUT'])
@jwt_required()
def update_rule(rule_id):
    rule = next((r for r in rules_data if r['id'] == rule_id), None)
    if rule:
        rule.update(request.json)
        return jsonify(rule)
    return jsonify({'message': 'Rule not found'}), 404

@app.route('/api/rules/<rule_id>', methods=['DELETE'])
@jwt_required()
def delete_rule(rule_id):
    global rules_data
    rules_data = [r for r in rules_data if r['id'] != rule_id]
    return '', 204

# Plugins endpoints
@app.route('/api/plugins')
def get_plugins():
    return jsonify(plugins_data)

@app.route('/api/plugins/<plugin_id>')
def get_plugin(plugin_id):
    plugin = next((p for p in plugins_data if p['id'] == plugin_id), None)
    if plugin:
        return jsonify(plugin)
    return jsonify({'message': 'Plugin not found'}), 404

@app.route('/api/plugins/<plugin_id>/enable', methods=['POST'])
@jwt_required()
def enable_plugin(plugin_id):
    plugin = next((p for p in plugins_data if p['id'] == plugin_id), None)
    if plugin:
        plugin['enabled'] = True
        return jsonify(plugin)
    return jsonify({'message': 'Plugin not found'}), 404

@app.route('/api/plugins/<plugin_id>/disable', methods=['POST'])
@jwt_required()
def disable_plugin(plugin_id):
    plugin = next((p for p in plugins_data if p['id'] == plugin_id), None)
    if plugin:
        plugin['enabled'] = False
        return jsonify(plugin)
    return jsonify({'message': 'Plugin not found'}), 404

# Events endpoints
@app.route('/api/events')
def get_events():
    n = int(request.args.get('n', len(events_history)))
    return jsonify(list(events_history)[-n:])

# System status endpoint
@app.route('/api/status')
def get_system_status():
    return jsonify({
        'status': 'healthy',
        'uptime': '2d 14h 32m',
        'sensors': {
            'total': len(sensors_data),
            'healthy': len([s for s in sensors_data if s['status'] == 'healthy']),
            'failed': len([s for s in sensors_data if s['status'] == 'failed'])
        },
        'plugins': {
            'total': len(plugins_data),
            'enabled': len([p for p in plugins_data if p['enabled']]),
            'disabled': len([p for p in plugins_data if not p['enabled']])
        },
        'rules': {
            'total': len(rules_data),
            'enabled': len([r for r in rules_data if r['enabled']]),
            'disabled': len([r for r in rules_data if not r['enabled']])
        },
        'telemetry': {
            'messages': len(telemetry_history),
            'last_received': telemetry_history[-1]['received_at'] if telemetry_history else None
        },
        'alerts': {
            'total': len(alerts_history),
            'recent': len([a for a in alerts_history if
                          datetime.fromisoformat(a['received_at']) > datetime.now() - timedelta(hours=24)])
        }
    })

# Error handlers
@app.errorhandler(404)
def not_found(error):
    return jsonify({'message': 'Endpoint not found'}), 404

@app.errorhandler(500)
def internal_error(error):
    return jsonify({'message': 'Internal server error'}), 500

@jwt.expired_token_loader
def expired_token_callback(jwt_header, jwt_payload):
    return jsonify({'message': 'Token has expired'}), 401

@jwt.invalid_token_loader
def invalid_token_callback(error):
    return jsonify({'message': 'Invalid token'}), 401

@jwt.unauthorized_loader
def missing_token_callback(error):
    return jsonify({'message': 'Authorization token required'}), 401

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True)
