import struct
import rclpy
from robot.steady_node import SteadyNode
from robot.serial_utils import open_serial_port
from enum import Enum
from msgs.msg import WheelSpeeds

#This uses robust serial, all explanations can be found on github
#https://github.com/araffin/arduino-robust-serial

BAUDRATE = 9600


class Order(Enum):
    """
    Pre-defined orders that are sent over to the arduino
    """

    HELLO = 0
    ALREADY_CONNECTED = 1
    WHEELSPEEDS = 2


class Serial(SteadyNode):

    def __init__(self):
        super().__init__("serial")
        self.serial_file = open_serial_port(baudrate=BAUDRATE, timeout=None)

        self.create_subscription(WheelSpeeds, "/robot/wheels", self.send_wheel_speeds, 10)

    def send_wheel_speeds(self, msg: WheelSpeeds):
        """Sends the wheel speeds in the WheelSpeeds msg over to the arduino"""
        self.write_order(Order.WHEELSPEEDS)
        self.write_i8(msg.front_left_wheel_speed)
        self.write_i8(msg.front_right_wheel_speed)
        self.write_i8(msg.back_right_wheel_speed)
        self.write_i8(msg.back_left_wheel_speed)

    def write_i8(self, value: int):
        if -128 <= value <= 127:
            self.serial_file.write(struct.pack('<b', value))
        else:
            print("Value error:{}".format(value))

    def write_i16(self, value):
        self.serial_file.write(struct.pack('<h', value))

    def write_order(self, order: Order):
        self.write_i8(order.value)


def main():
    rclpy.init()
    node = Serial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
