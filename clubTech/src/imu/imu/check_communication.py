"""
Ce script est un outil de diagnostic pour tester la communication entre l'IMU et la RaspberryPi à laquelle il est connecté.
Il effectue 4 étapes principales :
- Réinitialisation matérielle
- Initialisation du protocole SHTP
- Activation des capteurs
- Boucle de lecture
"""

import serial
import time
import os
import sys

# Import de la librairie SHTP
try:
    from adafruit_bno08x.uart import BNO08X_UART
    from adafruit_bno08x import (
        BNO_REPORT_LINEAR_ACCELERATION,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_ROTATION_VECTOR
    )
except ImportError:
    print("ERREUR : Librairie manquante.")
    sys.exit(1)

# Configuration
PORT = '/dev/ttyAMA0'
BAUD = 3000000  
RESET_GPIO_CHIP = 4
RESET_GPIO_LINE = 4

# Fonction pour effectuer un reset hardware
def hardware_reset():
    print(f"[1/3] Reset Hardware (Chip {RESET_GPIO_CHIP}, Line {RESET_GPIO_LINE})...")
    try:        
        os.system(f"gpioset {RESET_GPIO_CHIP} {RESET_GPIO_LINE}=0")
        time.sleep(0.1)
        os.system(f"gpioset {RESET_GPIO_CHIP} {RESET_GPIO_LINE}=1")
        print("      Attente du redémarrage du capteur (0.7s)...")
        time.sleep(0.7)     
    except Exception as e:
        print(f"❌ Erreur lors du Reset : {e}")
        sys.exit(1)

# Fonction principale
def main():
    print("=== TEST DE VALIDATION SHTP (Gyro + Accel X) ===")
    
    # Reset hardware
    hardware_reset()

    print(f"[2/3] Ouverture du port {PORT} à {BAUD} bauds...")
    try:   
        uart = serial.Serial(PORT, BAUD, timeout=0.1)   
    except Exception as e:
        print(f"❌ Impossible d'ouvrir le port série : {e}")
        return

    # Initialisation du protocole SHTP
    print("[3/3] Tentative de Handshake SHTP...")
    try:
        bno = BNO08X_UART(uart)
        print("✅ SUCCÈS ! Handshake réussi.")
    except Exception as e:
        print(f"\n❌ ÉCHEC DU HANDSHAKE : {e}")
        return

    # Activation des capteurs
    print("\n--- Activation : Rotation, Gyro, Accel Linéaire ---")
    try:
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION) 
    except Exception as e:
        print(f"Erreur activation : {e}")
    
    
    # Boucle de lecture
    print("Lecture en cours... Bougez le robot d'avant en arrière (Axe X) !")
    print("-" * 75)
    
    start_time = time.time()
    try:
        while True:
            # Récupération des données
            quat = bno.quaternion
            gyro = bno.gyro
            lin_accel = bno.linear_acceleration
            
            status_str = ""
            
            # 1. Quaternion (Juste pour vérifier que ça tourne)
            if quat:
                status_str += f"QuatW: {quat[3]:.2f} "
            
            # 2. Gyro Z (Vitesse de rotation)
            if gyro:
                status_str += f"| GyroZ: {gyro[2]:.3f} "
            
            # 3. Accélération X (Mouvement avant/arrière)
            if lin_accel:
                # lin_accel est un tuple (x, y, z). On prend l'index 0.
                accel_x = lin_accel[0] 
                status_str += f"| AccelX: {accel_x:.3f} m/s²"

            if status_str:
                print(f"t+{time.time()-start_time:.1f}s : {status_str}")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nTest terminé.")

if __name__ == "__main__":
    main()