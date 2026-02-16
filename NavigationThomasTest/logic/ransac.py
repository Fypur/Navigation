import numpy as np
import random
import matplotlib.pyplot as plt
import socket


def polar_to_cartesian(angle, distance):
    """Convert polar coordinates to Cartesian coordinates."""
    x = distance * np.cos(angle)
    y = distance * np.sin(angle)
    return x, y

def least_squares_line(points):
    """
    Fit a line using the least squares method.
    Returns the slope (m) and intercept (b) of the line (y = mx + b).
    """
    x = points[:, 0]
    y = points[:, 1]
    A = np.vstack([x, np.ones_like(x)]).T
    m, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return m, b

def distance_to_line(point, line_params):
    """
    Calculate the perpendicular distance of a point to a line.
    Line is defined as y = mx + b.
    """
    m, b = line_params
    x, y = point
    return abs(m * x - y + b) / np.sqrt(m**2 + 1)

def ransac(lidar_measurements, distance_limite=0.1, inlier_ratio=0.3, max_iterations=100):

    # Convert lidar measurements to Cartesian coordinates
    points =np.array([polar_to_cartesian(angle, dist) for angle, dist in lidar_measurements])

    detected_lines = []
    remaining_points = points.copy()

    for _ in range(max_iterations):

        # Randomly select two points to define a line
        sample_indices = random.sample(range(len(remaining_points)), 2)
        p1, p2 = remaining_points[sample_indices]

        # Compute line parameters using least squares
        sampled_points = np.array([p1, p2])
        m, b = least_squares_line(sampled_points)

        # Find inliers
        inliers = []
        for point in remaining_points:
            if distance_to_line(point, (m, b)) < distance_limite:
                inliers.append(point)
        inliers = np.array(inliers)

        # Check if the inlier count meets the ratio threshold
        if len(inliers) / len(points) >= inlier_ratio:
            # Fit a line to all inliers using least squares
            m, b = least_squares_line(inliers)

            # Save the detected line
            detected_lines.append({'m': m, 'b': b, 'inliers': inliers.tolist()})

            # Remove inliers from remaining points
            remaining_points = np.array([p for p in remaining_points if p.tolist() not in inliers.tolist()])

    return detected_lines

#create socket client to receive data
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server_socket:
    server_socket.bind(('localhost', 8080))
    print("Waiting for server...")
    i=0
    while i==0:
        i=1
        prev_angle=0
        angle=0
        L=[]
        while abs(angle-prev_angle)<2: #loops until the 360° information has been received
            data=server_socket.recvfrom(1024) 
            str_data = data[0].decode("utf-8")[3:].split(" ") #converts data to string and splits it between types of data
            prev_angle=angle
            angle=float(str_data[1])*np.pi/180 #extract angle from data
            dist=float(str_data[3]) #extract distance from data
            if dist > 0: #filter measurement errors
                L.append([angle,dist])
        
        lines = ransac(L, 100, 0.14, max_iterations=1000) #applies ransac algorithm




plt.figure(figsize=(8, 6))

# Plot all points
points = np.array([polar_to_cartesian(angle, dist) for angle, dist in L])
plt.scatter(points[:, 0], points[:, 1], color='blue', label='All Points')
x_limits = plt.xlim()
y_limits = plt.ylim()

# Plot detected lines
x_range = np.linspace(min(points[:, 0]) - 1, max(points[:, 0]) + 1, 100)
for i, line in enumerate(lines):
    m, b = line['m'], line['b']
    y_range = m * x_range + b
    plt.plot(x_range, y_range, label=f'Line {i + 1}', linewidth=2)

    # Highlight inliers
    inliers = np.array(line['inliers'])
    plt.scatter(inliers[:, 0], inliers[:, 1], label=f'Inliers Line {i + 1}', s=50, edgecolor='k')

plt.xlim(x_limits)
plt.ylim(y_limits)

plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.grid(True)
plt.legend()
plt.xlabel('X (meters)')
plt.ylabel('Y (meters)')
plt.title('RANSAC Line Fitting')
plt.show()
