# /home/pi/Bureau/Final_Project/rplidar_sdk-master/output/Linux/Release/lidar_test_usb0

import socket
import time
import numpy as np
import matplotlib.pyplot as plt

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
    server_socket.bind(('localhost', 8080))
    print("Waiting for server...")


    def get_data():
        measure = []
        data=server_socket.recvfrom(1024)
        str_data = data[0].decode("utf-8")[3:].split(" ")
        angle = float(str_data[1])
        distance = float(str_data[3])
        start_time = time.time()
        while (time.time() - start_time < 1):
            data=server_socket.recvfrom(1024)
            str_data = data[0].decode("utf-8")[3:].split(" ")
            angle = float(str_data[1])
            distance = float(str_data[3])
            measure.append((-angle,distance))
        return measure
 
    data = get_data()
    with open("data.txt", "a") as doc_data:
        for (angle,distance) in data:
            doc_data.write(f"{angle}    {distance}\n")
    print("End")
    
angles = np.radians([point[0] for point in data])
distances = np.array([point[1] for point in data])
x = distances * np.cos(angles+np.pi/2)
y = distances * np.sin(angles+np.pi/2)
max_distance = 300
plt.figure(figsize=(6, 6))
plt.scatter(x, y, color='blue', s=1)
plt.xlim(-max_distance, max_distance)
plt.ylim(-max_distance, max_distance)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.grid(True, linestyle='--', linewidth=0.5)
plt.title("Detected obstacles")
plt.xlabel("x (mm)")
plt.ylabel("y (mm) (direction pointed by the LiDAR)")
plt.savefig("360_scan.png")
plt.show()


