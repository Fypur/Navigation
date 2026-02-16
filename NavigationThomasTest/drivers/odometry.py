import RPi.GPIO as GPIO
import time
import json
import math
import sys
import os
import logging

# Import config
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import paho.mqtt.client as mqtt
from config import constants

# Logger
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Variables Globales de Position
x = 0.0
y = 0.0
theta = 0.0
last_time = time.time()

# Compteurs de ticks (signés)
ticks = [0, 0, 0, 0] # M1, M2, M3, M4

# Mapping des pins pour la boucle
encoders = [
    {"id": 0, "clk": constants.ENC_M1[0], "dt": constants.ENC_M1[1]},
    {"id": 1, "clk": constants.ENC_M2[0], "dt": constants.ENC_M2[1]},
    {"id": 2, "clk": constants.ENC_M3[0], "dt": constants.ENC_M3[1]},
    {"id": 3, "clk": constants.ENC_M4[0], "dt": constants.ENC_M4[1]}
]

# --- GESTION INTERRUPTIONS ---
def interrupt_callback(channel):
    global ticks
    # On cherche quel moteur a déclenché l'interruption
    for enc in encoders:
        if channel == enc["clk"]:
            clk_state = GPIO.input(enc["clk"])
            dt_state = GPIO.input(enc["dt"])
            
            # Si DT != CLK à la montée du CLK, on tourne dans un sens, sinon l'autre.
            # Note: J'assume ici que l'interruption est sur RISING edge.
            if dt_state != clk_state:
                ticks[enc["id"]] += 1
            else:
                ticks[enc["id"]] -= 1
            return

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    for enc in encoders:
        GPIO.setup(enc["clk"], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(enc["dt"], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
        # On attache une interruption sur la pin CLK
        # bouncetime=1 permet d'éviter les faux rebonds (bruit électrique)
        GPIO.add_event_detect(enc["clk"], GPIO.RISING, callback=interrupt_callback) # bouncetime=1 enlevé pour test vitesse

def run_odometry_service():
    global x, y, theta, last_time, ticks
    
    setup_gpio()
    
    client = mqtt.Client()
    try:
        client.connect(constants.MQTT_BROKER, constants.MQTT_PORT, 60)
        client.loop_start()
        logging.info("Service Odométrie démarré.")
    except Exception as e:
        logging.error(f"Erreur MQTT: {e}")
        return

    # Memorisation des ticks précédents pour calculer le delta
    last_ticks = [0, 0, 0, 0]

    try:
        while True:
            current_time = time.time()
            dt = current_time - last_time
            
            # On publie à 10Hz (toutes les 0.1s)
            if dt >= 0.1:
                # 1. Calcul des différences de ticks (combien on a bougé en 0.1s)
                delta_ticks = [c - l for c, l in zip(ticks, last_ticks)]
                last_ticks = list(ticks) # Copie
                
                # 2. Conversion en distance (mètres)
                d_wheels = [t * constants.METERS_PER_TICK for t in delta_ticks]
                
                # 3. Cinématique Robot 4 roues
                # On fait la moyenne Gauche vs Droite
                # Hypothèse: M1/M3 sont à Gauche, M2/M4 à Droite (A vérifier sur robot)
                # Si robot tourne à l'envers sur la carte, inverser ces groupes.
                dist_left = (d_wheels[0] + d_wheels[2]) / 2
                dist_right = (d_wheels[1] + d_wheels[3]) / 2
                
                # Distance linéaire parcourue (au centre)
                d_center = (dist_left + dist_right) / 2
                
                # Rotation (Delta Theta)
                d_theta = (dist_right - dist_left) / constants.ROBOT_WIDTH
                
                # 4. Intégration (Mise à jour de la position globale)
                # Si d_theta est petit, on approxime par une ligne droite
                theta += d_theta
                
                # Normalisation de l'angle entre -Pi et Pi
                theta = math.atan2(math.sin(theta), math.cos(theta))
                
                x += d_center * math.cos(theta)
                y += d_center * math.sin(theta)
                
                # 5. Envoi MQTT
                payload = {
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "theta": round(theta, 3),
                    "v_lin": round(d_center / dt, 2), # Vitesse linéaire m/s
                    "v_ang": round(d_theta / dt, 2)   # Vitesse angulaire rad/s
                }
                
                client.publish(constants.TOPIC_ODOM, json.dumps(payload))
                last_time = current_time
            
            time.sleep(0.01)

    except KeyboardInterrupt:
        logging.info("Arrêt Odométrie.")
        GPIO.cleanup()
        client.loop_stop()
        client.disconnect()

if __name__ == '__main__':
    run_odometry_service()