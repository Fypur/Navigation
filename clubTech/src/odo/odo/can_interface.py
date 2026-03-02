from math import pi
import time
import can

class CANInterface():
    """
    Interface pour gérer la communication avec le bus CAN.
    """
    
    def __init__(self, channel="can1", bit_rate=1000000):
        self.bus = can.Bus(interface="socketcan", channel=channel, index=1, bitrate=bit_rate)

    def get_last_msg(self):
        """
        Vide le buffer et retourne UNIQUEMENT le message le plus récent.
        Si aucun message, retourne None.
        """
        
        last_valid_data = None
        
        # On boucle tant qu'il y a des messages dans le tuyau (timeout=0 => non bloquant)
        while True:
            msg = self.bus.recv(timeout=0) 
            if msg is None:
                break # Plus de messages en attente
            
            # Vérification de l'ID 
            if msg.arbitration_id == 0x124:
                last_valid_data = msg.data

        if last_valid_data is None:
            return None
        
        # Extraction de la valeur (4 premiers octets)
        raw_value = int.from_bytes(last_valid_data[0:4], byteorder='big', signed=False)
        
        return raw_value

    # Méthode pour fermer le bus CAN
    def shutdown(self):
        self.bus.shutdown()

if __name__=="__main__" :
    import time as t
    base = CANInterface()
    msg = base.get_last_msg()
    distance = int.from_bytes(msg.data[1:7], byteorder='big', signed=True) / 16384 * 0.075 * pi
    print(f"Distance parcourue : {distance}")
    base.shutdown()
