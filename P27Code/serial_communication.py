from __future__ import division, print_function
import threading
import time
from RPi import GPIO
import math
import logging
import signal
import numpy as np
from time import sleep
from picamera import PiCamera
from encoder import display_encoder

try:
    import struct
except:
    pass
try:
    import queue
except ImportError:
    import Queue as queue

from robust_serial import write_order, Order, write_i8, write_i16, read_i16, read_i32, read_i8
from robust_serial.utils import open_serial_port
from constants import BAUDRATE
from measurement import increment_per_second

serial_file = None


def connect_to_arduino():
    global serial_file
    try:
        # Open serial port (for communication with Arduino)
        serial_file = open_serial_port(baudrate=BAUDRATE)
    except Exception as e:
        print('exception')
        raise e

    is_connected = False
    # Initialize communication with Arduino
    while not is_connected:
        print("Trying connection to Arduino...")
        write_order(serial_file, Order.HELLO)
        bytes_array = bytearray(serial_file.read(1))
        if not bytes_array:
            time.sleep(2)
            continue
        byte = bytes_array[0]
        if byte in [Order.HELLO.value, Order.ALREADY_CONNECTED.value]:
            is_connected = True

    time.sleep(2)
    c = 1
    while (c != b''):
        c = serial_file.read(1)


def update_motors_order_thread(L): #Send order and speeds to arduino
    global serial_file, order, commands, obstacle
    order = L[0]
    try:
        commands = [L[1], L[2], L[3], L[4]]
    except:
        commands = [0, 0, 0, 0]   
    #if order == Order.MOTOR and obstacle is True and commands[0] > 0 and commands[1] > 0 and commands[2] > 0 and commands[3] > 0:
     #   return #If we want it to not move after removing obstacle then change command and order to stop

    #print(L)
    write_order(serial_file, L[0])
    for i in L[1:]:
        write_i8(serial_file, i)  
    L.clear()


def read_sensor(): #Read ultrasonic sensor, returns distance
    write_order(serial_file, Order.ULTRASONIC)
    while True:
        try:
            g = read_i16(serial_file)
            return g
        except struct.error:
            print("case 1")
        except TimeoutError:
            print("case 2")
            write_order(serial_file, Order.ULTRASONIC)

def read_ADC(): #Read the average current in motor 2
    write_order(serial_file, Order.CURRENT)
    while True:
        try:
            i = read_i16(serial_file)
            return i
        except struct.error:
            print("case 1")
        except TimeoutError:
            print("case 2")
            write_order(serial_file, Order.CURRENT)

class PIDController:
    def __init__(self, K, Kp, Ki, Kd):
        self.K = K
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, measured_value, setpoint):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.prev_error
        output = measured_value + self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        #print("Kp * error=",int(self.Kp * error),"Ki * integral=",int(self.Ki * self.integral),"Kd * derivative=",int(self.Kd * derivative))
        #print("output=",output)
        self.prev_error = error
        return output * self.K

K = 0.05
Kp = 0.7
Ki = 0.0001
Kd = 0.1

pid_controller1 = PIDController(K, Kp, Ki, Kd)
pid_controller2 = PIDController(K, Kp, Ki, Kd)
pid_controller3 = PIDController(K, Kp, Ki, Kd)
pid_controller4 = PIDController(K, Kp, Ki, Kd)


def read_com_from_arduino(): #Close loop control
    return()
    global order, commands, serial_file, obstacle
    order = None
    moving = False
    obstacle = False
    counter_obstacle = False
    setpoint = 1000 #1300
    setpoint1, setpoint2, setpoint3, setpoint4 = 1000, 1000, 1000, 1000
    min_ditance = 50
    max_distance = 1000
    time_step = 0
    stop_time = 2
    turn_time = 0.9
    forwardtime = 1
    sensor_distance_list = [max_distance for i in range(5)]
    while True:
        #Read distance and make average to avoid sensor mistakes
        sensor_distance = read_sensor()
        sensor_distance_list.append(sensor_distance)
        sensor_distance_list = sensor_distance_list[1:]
        if (sensor_distance_list[0] <= min_ditance and sensor_distance_list[-1] <= min_ditance) or (sensor_distance_list[0] > max_distance and sensor_distance_list[-1] > max_distance):
            obstacle = True
            if moving is True:
                if counter_obstacle is False:
                    obstacle_time = time.time()
                    counter_obstacle = True
        else:
            obstacle = False         
        
        #If we order movement
        if order == Order.MOTOR:
            moving = True
            rs1, rs2, rs3, rs4 = increment_per_second()
            #print("rs1=",int(rs1),"rs2=",int(rs2),"rs3=",int(rs3),"rs4=",int(rs4))
            if counter_obstacle is False: #If no obstacle detected then proceed as normal with command
                setpoint1, setpoint2, setpoint3, setpoint4 = np.sign(commands[0])*setpoint, np.sign(commands[1])*setpoint, np.sign(commands[2])*setpoint, np.sign(commands[3])*setpoint
            elif counter_obstacle is True: #If there is obstacle enter the set of instructions to avoid obstacle
                if time.time() - obstacle_time <= 2: #stop
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                elif 2 < time.time() - obstacle_time <= 3.02: #left
                    setpoint1, setpoint2, setpoint3, setpoint4 = setpoint, -setpoint, setpoint, -setpoint
                elif 3.02 < time.time() - obstacle_time <= 4.8: #stop
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                elif 4.8 < time.time() - obstacle_time <= 5.84: #forward
                    setpoint1, setpoint2, setpoint3, setpoint4 = setpoint, setpoint, setpoint, setpoint
                elif 5.84 < time.time() - obstacle_time <= 7.5: #stop
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                elif 7.5 < time.time() - obstacle_time <= 8.459: #right
                    setpoint1, setpoint2, setpoint3, setpoint4 = -setpoint, setpoint, -setpoint, setpoint
                elif 8.459 < time.time() - obstacle_time <= 10.2: #stop
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                elif 10.2 < time.time() - obstacle_time <= 11.74: #forward
                    setpoint1, setpoint2, setpoint3, setpoint4 = setpoint, setpoint, setpoint, setpoint
                elif 11.74 < time.time() - obstacle_time <= 12.9: #stop
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                elif 12.9 < time.time() - obstacle_time <= 13.92: #right
                    setpoint1, setpoint2, setpoint3, setpoint4 = -setpoint, setpoint, -setpoint, setpoint
                elif 13.92 < time.time() - obstacle_time <= 15.7: #stop
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                elif 15.7 < time.time() - obstacle_time <= 16.7: #forward
                    setpoint1, setpoint2, setpoint3, setpoint4 = setpoint, setpoint, setpoint, setpoint
                elif 16.7 < time.time() - obstacle_time <= 18.5: #stop
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                elif 18.5 < time.time() - obstacle_time <= 19.4: #left
                    setpoint1, setpoint2, setpoint3, setpoint4 = setpoint, -setpoint, setpoint, -setpoint
                elif 19.4 < time.time() - obstacle_time <= 21:
                    setpoint1, setpoint2, setpoint3, setpoint4 = 0, 0, 0, 0
                    update_motors_order_thread([order, int(0), int(0), int(0), int(0)])
                    pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                else: #Go back to normal operation
                    setpoint1, setpoint2, setpoint3, setpoint4 = setpoint, setpoint, setpoint, setpoint
                    counter_obstacle = False
            #Update commands with PID function
            commands[0] = int(max(min(pid_controller1.update(rs1, setpoint1), 100), -100))
            commands[1] = int(max(min(pid_controller2.update(rs2, setpoint2), 100), -100))
            commands[2] = int(max(min(pid_controller3.update(rs3, setpoint3), 100), -100))
            commands[3] = int(max(min(pid_controller4.update(rs4, setpoint4), 100), -100))
            #Send values to arduino
            update_motors_order_thread([order, int(commands[0]), int(commands[1]), int(commands[2]), int(commands[3])])
        elif order == Order.STOP: #Stop command (p)
            if moving is True:
                commands = [0,0,0,0]
                update_motors_order_thread([order, int(commands[0]), int(commands[1]), int(commands[2]), int(commands[3])])
                pid_controller1.integral,pid_controller2.integral,pid_controller3.integral,pid_controller4.integral = 0,0,0,0
                moving = False
                counter_obstacle = False
        
        
