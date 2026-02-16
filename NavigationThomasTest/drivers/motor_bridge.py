import time
import json
import logging
import sys
import os

# Ajout du chemin parent pour pouvoir importer config et utils
sys.path.append(os.path.abspath(os.path.join(os.dirname(__file__), '..')))

import paho.mqtt.client as mqtt
from config import constants
from robust_serial import write_order, Order, write_i8
from robust_serial.utils import open_serial_port

# -- VARIABLES GLOBALES --
serial_file = None
last_command_time = time.time()

# -- CONFIGURATION DU LOGGER --
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# -- GESTION ARDUINO --

def connect_to_arduino():
    """Etablit la connexion série avec l'Arduino"""
    global serial_file
    try:
        logging.info(f"Tentative de connexion à l'Arduino sur {constants.SERIAL_PORT}...")
        serial_file = open_serial_port(baudrate=constants.BAUDRATE, port=constants.SERIAL_PORT)
    except Exception as e:
        logging.error(f"impossible d'ouvrir le port série : {e}")
        sys.exit(1)
    
    is_connected = False
    while not is_connected:
        logging.info("En attente du handshake Arduino...")
        write_order(serial_file, Order.HELLO)
        bytes_array = bytearray(serial_file.read(1))
        if not bytes_array:
            time.sleep(1)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            is_connected = True
            logging.info("Arduino connecté")


def send_motor_command(m1,m2,m3,m4):
    """Envoie les vitesses brutes (0-100) aux 4 moteurs"""
    try:
        # S'assurer que les valeurs sont entre -100 et 100 (limites des bytes signés/sécurité)
        m1 = max(min(int(m1), 100), -100)
        m2 = max(min(int(m2), 100), -100)
        m3 = max(min(int(m3), 100), -100)
        m4 = max(min(int(m4), 100), -100)

        # Envoi via robust_serial
        write_order(serial_file, Order.MOTOR)
        write_i8(serial_file, m1)
        write_i8(serial_file, m2)
        write_i8(serial_file, m3)
        write_i8(serial_file, m4)
    
    except Exception as e:
        logging.error(f"Erreur lors de l'envoi série : {e}")


def stop_robot():
    """Fonction d'arrêt d'urgence"""
    write_order(serial_file, Order.STOP)


# -- GESTION MQTT --

def on_connect(client, userdata, flags, rc):
    logging.info(f"Connecté au Broker MQTT avec le code : {rc}")
    # Abonnement au topic des commandes moteurs
    client.subscribe(constants.TOPIC_MOTOR_CMD)

def on_message(client, userdata, msg):
    """
    Reçoit un message JSON du type {"m1": 50, "m2": 50, "m3": 50, "m4" :50}
    Ou simplifié : {"linear": 50} (avancer tout droit)
    """
    global last_command_time
    last_command_time = time.time() #On reset le timer de sécurité

    try:
        payload = json.loads(msg.payload.decode())

        # Cas 1 : Commande directe des 4 moteurs
        if "m1" in payload:
            send_motor_command(payload['m1'], payload['m2'], payload['m3'], payload['m4'])

        # Cas 2 : Commande simple (Avancer/Reculer)
        elif "linear" in payload:
            val = payload['linear']
            send_motor_command(val, val, val, val)
        
        # Cas 3 : Arrêt explicite
        elif "stop" in payload:
            stop_robot()
    
    except Exception as e:
        logging.error(f"Erreur JSON : {e}")


# -- BOUCLE PRINCIPALE --

if __name__ == '__main__':

    # Connexion Arduino
    connect_to_arduino()

    # Configuration MQTT
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(constants.MQTT_BROKER, constants.MQTT_PORT, 60)
    except Exception as e:
        logging.error(f"Impossible de se connecter au MQTT : {e}")
        sys.exit(1)

    # Démarrage du thread MQTT
    client.loop_start()

    try:
        while True:
            # Si on n'a pas reçu d'ordre depuis 1 seconde, on arrête le robot
            if time.time() - last_command_time > 1.0:
                #A decommenter lorsque le robot roulera 
                #stop_robot()
                pass

            time.sleep(0.1)

    except KeyboardInterrupt:
        logging.info("Arrêt du programme...")
        stop_robot()
        client.loop_stop()
        client.disconnect()