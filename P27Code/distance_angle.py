import socket
import time

def distance_obstacle_angle(angle,duration = 1):
    
    "This function takes an angle (between 0 and 360°) as an argument"
    "and returns the distance in mm between the lidar and the distance between the lidar"
    "and the obstacle located at the given angle."
    "It is possible to measure the data acquisition time with the duration variable in seconds."
    
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket: #on récupère les couples (angle,distance) et
        server_socket.bind(('localhost', 8080)) #on modifie l'angle de ces points de sorte que le point voulu ait un angle 

        def get_closest_distance(duration,angle_):   #proche de 0° (on cherche juste une distance ici pour un angle donné)
            points = []                      #ensuite on récupère la distance qui est associée à l'angle le plus proche de 0°
            start_time = time.time()      #en triant la liste des points

            while time.time() - start_time < duration:
                data = server_socket.recvfrom(1024)
                str_data = data[0].decode("utf-8")[3:].split(" ")
                angle = float(str_data[1])
                distance = float(str_data[3])
                if distance > 1:  # 1mm Ignore les très proches souvent associés à une distance de 0 i.e. erreur de mesure 
                    points.append((abs((abs(angle)-angle_)), distance))

            # Trie les points par angle le plus proche de 0°
            points.sort(key=lambda x: x[0])

            # Cherche le premier avec une distance non nulle
            for _, distance in points:
                if distance > 0:  #pas nécessaire car distance >1mm
                    return distance
            return None

        closest_distance = get_closest_distance(duration,angle)
        return closest_distance