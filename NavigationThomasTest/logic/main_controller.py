import sys
import os
import json
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import paho.mqtt.client as mqtt
from config import constants
from logic import obstacle_avoidance

# État par défaut
current_mode = "MANUAL"

def on_connect(client, userdata, flags, rc):
    print("Cerveau connecté au MQTT.")
    client.subscribe(constants.TOPIC_MODE)        # Écoute le changement de mode
    client.subscribe(constants.TOPIC_MANUAL_CMD)  # Écoute le joystick
    client.subscribe(constants.TOPIC_LIDAR)

def on_message(client, userdata, msg):
    global current_mode
    
    try:
        payload = json.loads(msg.payload.decode())

        # 1. Si c'est un changement de mode
        if msg.topic == constants.TOPIC_MODE:
            payload = json.loads(msg.payload.decode())
            new_mode = payload.get("mode", "MANUAL")
            
            if new_mode != current_mode:
                print(f"--- CHANGEMENT DE MODE : {new_mode} ---")
                current_mode = new_mode
                # Sécurité : on stop tout quand on change
                client.publish(constants.TOPIC_MOTOR_CMD, '{"linear": 0}')

        # 2. Si c'est un ordre de la télécommande
        elif msg.topic == constants.TOPIC_MANUAL_CMD:
            if current_mode == "MANUAL":
                # On transfère l'ordre directement aux moteurs
                client.publish(constants.TOPIC_MOTOR_CMD, json.dumps(payload))
            else:
                print("Ordre manuel ignoré (Mode AUTO actif)")

        # 3. Gestion AUTOMATIQUE (LiDAR)
        elif msg.topic == constants.TOPIC_LIDAR:
            if current_mode == "AUTO":
                # A. Décoder les données LiDAR
                scan_data = json.loads(msg.payload.decode())
                
                # B. Calculer la décision (via logic/obstacle_avoidance.py)
                command = obstacle_avoidance.get_autonomous_command(scan_data)
                
                # C. Envoyer l'ordre aux moteurs
                client.publish(constants.TOPIC_MOTOR_CMD, json.dumps(command))

    except Exception as e:
        print(f"Erreur dans le cerveau : {e}")

if __name__ == '__main__':
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        client.connect(constants.MQTT_BROKER, constants.MQTT_PORT, 60)
        client.loop_forever()
    except KeyboardInterrupt:
        print("Arrêt du cerveau.")