# all of this code is documented at https://github.com/Fypur/Navigation/wiki/Encoders

import rclpy
import rclpy.logging
from robot.steady_node import SteadyNode
from enum import IntEnum
import time
from msgs.msg import RPMs
import collections
from robot.robot_config import FRONT_LEFT_FLIPPED, FRONT_RIGHT_FLIPPED, BACK_RIGHT_FLIPPED, BACK_LEFT_FLIPPED, RPMS_UPDATING_PERIOD

# We use gpiozero for Raspberry Pi 5 compatibility, as it interfaces smoothly
# with the new RP1 chip using libgpiod under the hood.
try:
    from gpiozero import RotaryEncoder
except ImportError:
    rclpy.logging.get_logger("encoders").error(
        "Couldn't import gpiozero. Run: pip install gpiozero")
    exit()

class PinMap(IntEnum):
    ENCODER_FRONT_LEFT_A = 17
    ENCODER_FRONT_LEFT_B = 27
    ENCODER_FRONT_RIGHT_A = 20
    ENCODER_FRONT_RIGHT_B = 21
    ENCODER_BACK_RIGHT_A = 19
    ENCODER_BACK_RIGHT_B = 26
    ENCODER_BACK_LEFT_A = 2
    ENCODER_BACK_LEFT_B = 3

PULSES_PER_REV = 234.3


class Encoders(SteadyNode):

    class EncoderSignalPin:

        def __init__(self, a_pin: int, b_pin: int, reversed: bool, logger) -> None:
            self.a_pin = a_pin
            self.b_pin = b_pin
            self.reversed_motor = reversed
            self.sliding_average_window = 0.3
            self.logger = logger

            self.positive_encoder_pulse_timestamps = collections.deque()
            self.negative_encoder_pulse_timestamps = collections.deque()
            self.rpm = 0.0

            # Initialize RotaryEncoder. Pull-up is True by default.
            # max_steps=0 ensures the internal counter doesn't hit a ceiling.
            self.encoder = RotaryEncoder(a_pin, b_pin, max_steps=0)

            # Map the rotation events directly to our timestamp logging functions
            self.encoder.when_rotated_clockwise = self.on_forward
            self.encoder.when_rotated_counter_clockwise = self.on_backward

            logger.info(f"Pin setup for pins a={a_pin} b={b_pin} via gpiozero")

        def on_forward(self):
            self.positive_encoder_pulse_timestamps.append(time.time())

        def on_backward(self):
            self.negative_encoder_pulse_timestamps.append(time.time())

        def update_rpm(self):
            """
            Updates the RPM of the wheel associated with this encoder and returns it.
            The RPM is positive when spinning forwards, and negative when spinning backwards.
            RPM is calculated using a sliding average (moyenne glissante).
            """
            current_time = time.time()

            def remove_old_timestamps(encoder_pulse_timestamps: collections.deque):
                while encoder_pulse_timestamps and \
                      current_time - encoder_pulse_timestamps[0] > self.sliding_average_window:
                    encoder_pulse_timestamps.popleft()

            remove_old_timestamps(self.positive_encoder_pulse_timestamps)
            remove_old_timestamps(self.negative_encoder_pulse_timestamps)

            pulse_count = len(self.positive_encoder_pulse_timestamps) - len(self.negative_encoder_pulse_timestamps)
            self.rpm = 60 * pulse_count / (self.sliding_average_window * PULSES_PER_REV)

            if not self.reversed_motor:
                return self.rpm
            else:
                return -self.rpm

        def cancel(self):
            # Cleanly release the GPIO lines
            self.encoder.close()

    def __init__(self):
        super().__init__("encoders")

        def is_raspberry_pi():
            try:
                with open('/proc/device-tree/model', 'r') as f:
                    return 'Raspberry Pi' in f.read()
            except FileNotFoundError:
                return False

        if not is_raspberry_pi():
            self.get_logger().error("This node can only run on a raspberry pi !")
            exit()

        logger = self.get_logger()
        
        # We no longer need to pass a pi instance to the signal pins.
        self.encoderSignalPins = [
            self.EncoderSignalPin(PinMap.ENCODER_FRONT_LEFT_A, PinMap.ENCODER_FRONT_LEFT_B, FRONT_LEFT_FLIPPED, logger),
            self.EncoderSignalPin(PinMap.ENCODER_FRONT_RIGHT_A, PinMap.ENCODER_FRONT_RIGHT_B, FRONT_RIGHT_FLIPPED, logger),
            self.EncoderSignalPin(PinMap.ENCODER_BACK_RIGHT_A, PinMap.ENCODER_BACK_RIGHT_B, BACK_RIGHT_FLIPPED, logger),
            self.EncoderSignalPin(PinMap.ENCODER_BACK_LEFT_A, PinMap.ENCODER_BACK_LEFT_B, BACK_LEFT_FLIPPED, logger),
        ]

        self.pub = self.create_publisher(RPMs, "/robot/encoders", 10)
        self.create_timer(RPMS_UPDATING_PERIOD, self.send_RPMs)
        self.get_logger().info("Encoders node launched (Pi 5 Compatible)")

    def send_RPMs(self):
        msg = RPMs()
        msg.front_left_rpm = self.encoderSignalPins[0].update_rpm()
        msg.front_right_rpm = self.encoderSignalPins[1].update_rpm()
        msg.back_right_rpm = self.encoderSignalPins[2].update_rpm()
        msg.back_left_rpm = self.encoderSignalPins[3].update_rpm()

        self.get_logger().debug(
            f"measured RPMs: {msg.front_left_rpm:.1f}, {msg.front_right_rpm:.1f}, {msg.back_right_rpm:.1f}, {msg.back_left_rpm:.1f}"
        )
        self.pub.publish(msg)

    def destroy_node(self):
        for enc in self.encoderSignalPins:
            enc.cancel()
        super().destroy_node()


def main():
    rclpy.init()
    node = Encoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()