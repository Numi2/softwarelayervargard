import json
import time
import threading
from collections import deque

from flask import Flask, jsonify, Response, request
import paho.mqtt.client as mqtt

# Configuration
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
TELEMETRY_TOPIC = 'vargard/telemetry'
ALERTS_TOPIC = 'vargard/alerts'
MAX_HISTORY = 100

app = Flask(__name__)
telemetry_history = deque(maxlen=MAX_HISTORY)
alerts_history = deque(maxlen=MAX_HISTORY)

def on_connect(client, userdata, flags, rc):
    print(f"Dashboard MQTT connected (rc={rc})")
    client.subscribe([(TELEMETRY_TOPIC, 0), (ALERTS_TOPIC, 0)])

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode('utf-8'))
    except Exception:
        return
    if msg.topic == TELEMETRY_TOPIC:
        telemetry_history.append(data)
    elif msg.topic == ALERTS_TOPIC:
        alerts_history.append(data)

# Start MQTT client in a background thread
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
threading.Thread(target=client.loop_forever, daemon=True).start()

@app.route('/telemetry')
def get_telemetry():
    n = int(request.args.get('n', len(telemetry_history)))
    return jsonify(list(telemetry_history)[-n:])

@app.route('/alerts')
def get_alerts():
    n = int(request.args.get('n', len(alerts_history)))
    return jsonify(list(alerts_history)[-n:])

def stream_data(history):
    def generate():
        last_index = 0
        while True:
            while len(history) > last_index:
                item = history[last_index]
                yield f"data: {json.dumps(item)}\n\n"
                last_index += 1
            time.sleep(1)
    return Response(generate(), mimetype='text/event-stream')

@app.route('/telemetry/stream')
def stream_telemetry():
    return stream_data(telemetry_history)

@app.route('/alerts/stream')
def stream_alerts():
    return stream_data(alerts_history)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)