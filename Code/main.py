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
import struct
try:
    import queue
except ImportError:
    import Queue as queue

from robust_serial import Order
from serial_communication import connect_to_arduino,read_com_from_arduino
from command import *
from encoder import encoder,collect_encoder



def main():
    #test_camera()
    print('Activating encoder')
    thread1.start()
    print('Encoder activated')
    serial_file=connect_to_arduino()
    print('Connected to Arduino,reading its communication...')
    thread2.start()
    thread3.start()
    #thread4.start()
    print("Welcome to raspi_serial.py")
    print("Press enter to validate your commands")
    print("Enter h to get the list of valid commands")
    read_cmd()


# Create threads for each function
thread1 = threading.Thread(target=encoder)
thread2 = threading.Thread(target=read_cmd)
thread3 = threading.Thread(target=read_com_from_arduino)
#thread4 = threading.Thread(target=collect_encoder)


if __name__ == "__main__":
    main()