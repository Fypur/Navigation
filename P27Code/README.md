# S8_Navigation_P25

## Project organisation

This project consists of the following elements:

- `4_independant_wheels_slave/`: This directory houses the Arduino script necessary for serial communication.
  - `4_independant_wheels_slave.ino`: Script executed on the Arduino board to facilitate serial communication.
  - `parameter.h`: Contains essential constants utilized within the Arduino code.
  - `slave.h`: Module incorporating functions for reading the serial file.
  - `order.h`: Stores the values of Order for both Raspberry Pi boards.

- `robust_serial/`: This directory contains basic read and write commands for serial file manipulation.

![Reading Manual](readme_utils/read.gif)

The Python code is structured as follows:

I) Robot 

- `main.py`: Entry point of the program, allowing execution of commands on the robot.
- `serial_communication.py`: Module containing functions for communication with the Arduino board. Including all the close loop calculations and communication to the Romeo Board. It also contains the obstacle avoid necessary functions.
- `command.py`: Module for processing user commands.
- `encoder.py`: Module enabling encoder reading.
- `movement_type.py`: Module associating robot movements with motor commands.
- `measurement.py`: Module for converting data into speeds (rotation, translation, etc.).
- `constants.py`: File containing all the constants used across different files.

2) Lidar : obstacle detection

- 'lidar_test.py' File to test the lidar 
- 'ransac.py' file to map the environment with the ransac method (please see the rapport S6 2024/2025)
- 'map with convex hull.py' file to map the environment with the convex hull method
- 'distance_angle.py' File to measure the distance of the nearest obstacle at a given angle
- 'rotation.py' and 'mesure_angle_rotation.py' each file can calculate the angle of a rotation with the first method (please see the rapport S6 2024/2025)
- 'detection_obstacle.py', 'analyse_mouvement.py' and 'matrice_rotation___vecteur_translation.py' execute 'analyse_mouvement.py' to measure he angle of a rotation with the second method (please see the rapport S6 2024/2025)

3) encoder.py : the code allowing reading the encoders value
-the function 'return_speed' return the speed of the four wheels
-to change the period during which the results are filtered, change the length of w1 to w4 line 6 and update line 177 to 180

![Kung Fu Panda GIF](readme_utils/kung-fu-panda.gif)

4) training and testing
-uses the function 'test1' in the script 'entrainement.py' and call it with the 'en' instruction after running main to easily test code
## User Guide

![Difficult](readme_utils/think.gif)

1. Upload the `4_independant_wheels_slave.ino` code to the Romeo BLE (Arduino Uno).
2. Execute the main.py script on the Raspberry Pi. Upon execution, invoking the 'h' command will provide you with a comprehensive list of available commands that you can utilize.