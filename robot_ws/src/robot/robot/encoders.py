# all of this code is documented at https://github.com/Fypur/Navigation/wiki/Encoders

import rclpy
from robot.steady_node import SteadyNode
import RPi.GPIO as GPIO  #pip install RPi.GPIO
from enum import IntEnum
import time
from msgs.msg import RPMs
import collections


class PinMap(IntEnum):
    ENCODER_FRONT_LEFT_A = 17
    ENCODER_FRONT_LEFT_B = 27
    ENCODER_FRONT_RIGHT_A = 20
    ENCODER_FRONT_RIGHT_B = 21
    ENCODER_BACK_RIGHT_A = 19
    ENCODER_BACK_RIGHT_B = 26
    ENCODER_BACK_LEFT_A = 2
    ENCODER_BACK_LEFT_B = 3

class Encoders(SteadyNode):

    class EncoderSignalPin:

        def __init__(self, a_pin: int, b_pin: int, reversed: bool, logger) -> None:

            # pin setup
            self.a_pin = a_pin
            self.b_pin = b_pin
            self.reversed_motor = reversed
            self.sliding_average_window = 0.3 #we do a sliding average over 0.3s in order to keep the readings stable
            GPIO.setup(self.a_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(self.b_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            self.logger = logger
            logger.info(f"Pin setup for pins a {self.a_pin} and b {self.b_pin}")
            # the following line triggers self.encoder_pulse() when the bpin goes from HIGH to LOW,
            # so when the wheel spun by a certain amount
            GPIO.add_event_detect(self.b_pin,
                                  GPIO.FALLING,
                                  callback=(lambda channel: self.encoder_pulse()))

            self.positive_encoder_pulse_timestamps = collections.deque()  # modified when the encoder pulses, so when the wheel spins by a certain amount
            self.negative_encoder_pulse_timestamps = collections.deque()
            self.rpm = 0.

        def encoder_pulse(self):
            # self.logger.debug(f"pin {self.b_pin} encoder pulse")

            if GPIO.input(self.a_pin) == GPIO.LOW:
                self.positive_encoder_pulse_timestamps.append(time.time())
            else:
                self.negative_encoder_pulse_timestamps.append(time.time())

        def update_rpm(self):
            """
                Updates the RPM of the wheel associated with this encoder and returns it.
                The RPM is positive when spinning forwards, and negative when spinning backwards
            """

            currentTime = time.time()

            def remove_old_timestamps(encoder_pulse_timestamps: collections.deque):
                while len(encoder_pulse_timestamps) > 0:
                    if currentTime - encoder_pulse_timestamps[0] > self.sliding_average_window:
                        encoder_pulse_timestamps.popleft()
                    else:
                        break

            remove_old_timestamps(self.positive_encoder_pulse_timestamps)
            remove_old_timestamps(self.negative_encoder_pulse_timestamps)

            encoder_pulse_count = len(self.positive_encoder_pulse_timestamps) - len(self.negative_encoder_pulse_timestamps)
            self.rpm = 60000 * encoder_pulse_count / (self.sliding_average_window * 234.3)
            self.rpm = -self.rpm if self.reversed_motor else self.rpm

            return self.rpm


    def __init__(self):
        super().__init__("encoders")

        GPIO.setmode(GPIO.BCM)
        logger = self.get_logger()
        self.encoderSignalPins = [
            self.EncoderSignalPin(PinMap.ENCODER_FRONT_LEFT_A, PinMap.ENCODER_FRONT_LEFT_B, False, logger),
            self.EncoderSignalPin(PinMap.ENCODER_FRONT_RIGHT_A, PinMap.ENCODER_FRONT_RIGHT_B, True, logger),
            self.EncoderSignalPin(PinMap.ENCODER_BACK_RIGHT_A, PinMap.ENCODER_BACK_RIGHT_B, False, logger),
            self.EncoderSignalPin(PinMap.ENCODER_BACK_LEFT_A, PinMap.ENCODER_BACK_LEFT_B, False, logger),
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

        self.get_logger().debug(f"measured RPMS: {msg.front_left_rpm}, {msg.front_right_rpm}, {msg.back_right_rpm}, {msg.back_left_rpm}")

        self.pub.publish(msg)




def main():
    rclpy.init()
    GPIO.cleanup()
    try:
        rclpy.spin(Encoders())
    finally:
        GPIO.cleanup()
    rclpy.shutdown()
