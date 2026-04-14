# all of this code is documented at https://github.com/Fypur/Navigation/wiki/Encoders

import rclpy
from robot.steady_node import SteadyNode
import RPi.GPIO as GPIO  #pip install RPi.GPIO
from enum import IntEnum
import time
from msgs.msg import RPMs


class PinMap(IntEnum):
    ENCODER_FRONT_LEFT_A = 1
    ENCODER_FRONT_LEFT_B = 2
    ENCODER_FRONT_RIGHT_A = 3
    ENCODER_FRONT_RIGHT_B = 4
    ENCODER_BACK_RIGHT_A = 5
    ENCODER_BACK_RIGHT_B = 6
    ENCODER_BACK_LEFT_A = 7
    ENCODER_BACK_LEFT_B = 8

class Encoders(SteadyNode):

    class EncoderSignalPin:
        def __init__(self, a_pin: int, b_pin: int) -> None:

            # pin setup
            self.a_pin = a_pin
            self.b_pin = b_pin
            GPIO.setup(self.a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.b_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            # the following line triggers self.encoder_pulse() when the bpin goes from HIGH to LOW,
            # so when the wheel spun by a certain amount
            GPIO.add_event_detect(self.b_pin,
                                  GPIO.FALLING,
                                  callback=(lambda channel: self.encoder_pulse()))

            self.encoderPulseCount = 0 # modified when the encoder pulses, so when the wheel spins by a certain amount
            self.began_measure_time = 0.
            self.last_measured_time = 0.

        def encoder_pulse(self):
            if GPIO.input(self.a_pin) == GPIO.LOW:
                self.encoderPulseCount += 1
            else:
                self.encoderPulseCount -= 1
            self.last_measured_time = int(round(time.time() * 1000))

        def update_rpm(self):
            """
                Updates the RPM of the wheel associated with this encoder and returns it.
                The RPM is positive when spinning forwards, and negative when spinning backwards
            """
            if self.last_measured_time > self.began_measure_time:
                self.rpm1 = 60000 * self.encoderPulseCount / (self.last_measured_time - self.began_measure_time)
                self.rpm1 = round(self.rpm1 / 234.3, 2)
                self.began_measure_time = self.last_measured_time
                self.encoderPulseCount = 0

            return self.rpm1

    def __init__(self):
        super().__init__("encoders")

        self.encoderSignalPins = [
            self.EncoderSignalPin(PinMap.ENCODER_FRONT_LEFT_A, PinMap.ENCODER_FRONT_LEFT_B),
            self.EncoderSignalPin(PinMap.ENCODER_FRONT_RIGHT_A, PinMap.ENCODER_FRONT_RIGHT_B),
            self.EncoderSignalPin(PinMap.ENCODER_BACK_RIGHT_A, PinMap.ENCODER_BACK_RIGHT_B),
            self.EncoderSignalPin(PinMap.ENCODER_BACK_LEFT_A, PinMap.ENCODER_BACK_LEFT_B),
        ]

        self.pub = self.create_publisher(RPMs, "/robot/encoders", 10)

        self.create_timer(0.1, self.send_RPMs)

        self.get_logger().info("Encoders node launched")

    def send_RPMs(self):
        msg = RPMs()

        msg.front_left_rpm = self.encoderSignalPins[0].update_rpm()
        msg.front_right_rpm = self.encoderSignalPins[1].update_rpm()
        msg.back_right_rpm = self.encoderSignalPins[2].update_rpm()
        msg.back_left_rpm = self.encoderSignalPins[3].update_rpm()

        self.pub.publish(msg)




def main():
    rclpy.init()
    rclpy.spin(Encoders())
    rclpy.shutdown()
