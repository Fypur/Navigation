from flask import Flask, render_template, request, jsonify
import paho.mqtt.client as mqtt
import json
import sys
import os

# Configuration MQTT locale pour le serveur web
MQTT_BROKER = "localhost"
TOPIC_MANUAL = "robot/control/manual_cmd"
TOPIC_MODE = "robot/control/mode"

app = Flask(__name__)
client = mqtt.Client()
client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/move', methods=['POST'])
def move():
    # Reçoit les données du Joystick (x, y)
    data = request.json
    # Conversion simple Joystick -> Vitesse Moteur
    # data['x'] et data['y'] sont entre -50 et 50 (par exemple)
    
    # Pour faire simple : on avance selon Y
    linear_speed = int(data.get('y', 0)) * 2  # Facteur multiplicateur
    
    cmd = {"linear": linear_speed}
    client.publish(TOPIC_MANUAL, json.dumps(cmd))
    return jsonify({"status": "ok"})

@app.route('/set_mode', methods=['POST'])
def set_mode():
    data = request.json
    mode = data.get('mode', 'MANUAL')
    client.publish(TOPIC_MODE, json.dumps({"mode": mode}))
    return jsonify({"status": "ok"})

if __name__ == '__main__':
    # Écoute sur toutes les interfaces (0.0.0.0) pour être accessible via le Wi-Fi
    app.run(host='0.0.0.0', port=5000)