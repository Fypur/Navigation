import socket
import time
import json
import sys
import os
import logging
import subprocess
import signal

# Ajout du chemin parent pour importer la config
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import paho.mqtt.client as mqtt
from config import constants

# --- CONFIGURATION ---
PATH_TO_CPP_EXECUTABLE = "/home/pi/Bureau/NavigationProject/rplidar_sdk-master/output/Linux/Release/lidar_test_usb0"

UDP_IP = "localhost"
UDP_PORT = 8080
BUFFER_DURATION = 0.1  # Envoi MQTT à 10Hz

# Setup Logger
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def run_lidar_service():
    lidar_process = None
    sock = None
    client = mqtt.Client()

    try:
        # 1. Connexion MQTT
        try:
            client.connect(constants.MQTT_BROKER, constants.MQTT_PORT, 60)
            client.loop_start()
            logging.info("MQTT connecté.")
        except Exception as e:
            logging.error(f"Erreur MQTT (le broker est-il lancé ?) : {e}")
            return

        # 2. Préparation du Socket UDP (On écoute avant de lancer le C++)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((UDP_IP, UDP_PORT))
        sock.settimeout(1.0) # Timeout pour ne pas bloquer indéfiniment si le C++ plante
        logging.info(f"Socket UDP ouvert sur {UDP_PORT}")

        # 3. Lancement automatique du programme C++
        if os.path.exists(PATH_TO_CPP_EXECUTABLE):
            logging.info(f"Démarrage du driver C++ : {PATH_TO_CPP_EXECUTABLE}")
            # on lance le processus sans bloquer le script python
            lidar_process = subprocess.Popen([PATH_TO_CPP_EXECUTABLE])
            time.sleep(2) # On laisse 2 secondes au LIDAR pour se lancer
        else:
            logging.error(f"ERREUR : Fichier introuvable -> {PATH_TO_CPP_EXECUTABLE}")
            return

        scan_buffer = [] # pour récupérer les données scannées par LIDAR au cours de la durée
        last_send_time = time.time()
        logging.info("Service LiDAR opérationnel. En attente de données...")

        # 4. Boucle principale
        while True:
            try:
                # Réception UDP
                data, addr = sock.recvfrom(1024)
                
                # Décodage
                raw_str = data.decode("utf-8")
                parts = raw_str[3:].split() # On enelève le préfixe des data envoyées par LIDAR
                
                if len(parts) >= 4:
                    angle = float(parts[1])
                    distance = float(parts[3])
                    
                    if distance > 0:
                        scan_buffer.append({"a": angle, "d": distance})

                # Envoi MQTT régulier
                current_time = time.time()
                if current_time - last_send_time > BUFFER_DURATION:
                    if scan_buffer:
                        payload = json.dumps(scan_buffer)
                        client.publish(constants.TOPIC_LIDAR, payload)
                        scan_buffer = [] # Reset buffer
                        last_send_time = current_time

            except socket.timeout:
                # Si on ne reçoit rien pendant 1 seconde
                logging.warning("Pas de données LiDAR reçues depuis 1s...")
                # Vérifier si le processus C++ est mort
                if lidar_process.poll() is not None:
                    logging.error("Le programme C++ s'est arrêté")
                    break
            except Exception as e:
                logging.error(f"Erreur de boucle : {e}")
                continue

    except KeyboardInterrupt:
        logging.info("Arrêt demandé par l'utilisateur...")

    finally:
        # 5. NETTOYAGE
        logging.info("Nettoyage en cours...")
        
        # Tuer le processus C++ s'il tourne encore
        if lidar_process:
            logging.info("Arrêt du driver C++...")
            lidar_process.terminate()
            try:
                lidar_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                lidar_process.kill() # Force kill si ça résiste
        
        if sock:
            sock.close()
        
        client.loop_stop()
        client.disconnect()
        logging.info("Tout est éteint")

if __name__ == '__main__':
    run_lidar_service()