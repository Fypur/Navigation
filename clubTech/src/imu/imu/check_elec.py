"""
Ce script permet de tester le branchement de l'IMU.
Lorsque celle-ci démarre, elle envoie un message de réveil que l'on peut lire ici en brut.
Pour vérifier le branchement, faites un reset de l'IMU au moment indiqué et observez le message de réveil dans le terminal.
"""

import serial
import time

# Sur votre Raspberry Pi 5 avec Ubuntu
PORT = '/dev/ttyAMA0'
BAUD = 3000000

print(f"--- Écoute brute sur {PORT} à {BAUD} bauds ---")
print("Le script écoute... ")
print("👉 FAITES UN RESET MAINTENANT (Fil RST sur GND) pour voir le message de réveil.")
print("Ctrl+C pour quitter.")

try:
    # Timeout court pour ne pas bloquer
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f"Erreur critique d'ouverture du port : {e}")
    exit()

while True:
    try:
        # Si des octets sont en attente dans le tampon
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            # Affichage en hexadécimal (ex: 17 00 01 ...)
            print(f"REÇU ({len(data)} octets) : {data.hex(' ')}")
        
        time.sleep(0.01)
        
    except KeyboardInterrupt:
        print("\nArrêt.")
        break
    except Exception as e:
        print(f"Erreur de lecture : {e}")