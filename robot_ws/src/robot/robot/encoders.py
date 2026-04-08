# all of this code is documented at https://github.com/Fypur/Navigation/wiki/Encoders

import rclpy
from steady_node import SteadyNode
import RPi.GPIO as GPIO
from enum import IntEnum
import time

class PinMap(IntEnum):
    ENCODER1A = 1
    ENCODER1B = 2
    ENCODER2A = 3
    ENCODER2B = 4
    ENCODER3A = 5
    ENCODER3B = 6
    ENCODER4A = 7
    ENCODER4B = 8

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

        def get_rpm(self):
            """
                Gets the RPM of the wheel associated with this encoder
                The RPM is positive when spinning forwards, and negative when spinning backwards
            """
            if self.last_measured_time > self.began_measure_time:
                rpm1 = 60000 * self.encoderPulseCount / (self.last_measured_time - self.began_measure_time)
                rpm1 = round(rpm1 / 234.3, 2)
                self.began_measure_time = self.last_measured_time
                self.encoderPulseCount = 0

    def __init__(self):
        super().__init__("encoders")

        self.encoderSignalPins = [
            self.EncoderSignalPin(PinMap.ENCODER1A, PinMap.ENCODER1B),
            self.EncoderSignalPin(PinMap.ENCODER2A, PinMap.ENCODER2B),
            self.EncoderSignalPin(PinMap.ENCODER3A, PinMap.ENCODER3B),
            self.EncoderSignalPin(PinMap.ENCODER4A, PinMap.ENCODER4B),
        ]

        # self.pub = self.create_publisher(RPMs, "/robot/encoders", 10)

        self.get_logger().info("Health node launched")




def main():
    rclpy.init()
    rclpy.spin(Encoders())
    rclpy.shutdown()
