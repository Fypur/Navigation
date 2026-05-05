import rclpy
import math
import dearpygui.dearpygui as dpg
from robot.steady_node import SteadyNode
from msgs.msg import RPMs
import numpy as np

#this was written using AI, use it to look at what the lidar sees in real time

class AsservVisualizer(SteadyNode):

    class WheelPlot():
        def __init__(self, wheel_name : str) -> None:
            self.data_x = np.array([])
            self.data_y = np.array([])
            self.time = 0.0
            self.cmd_line = None

            with dpg.plot(label=wheel_name, height=200, width=-1):
                self.x_axis = dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)")
                self.y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="RPM")

                # Create an empty scatter series to hold our points
                self.line_series = dpg.add_line_series([], [], parent=self.y_axis)

        def add_data(self, rpm):
            self.time += 0.1 # Assuming 10hz
            self.data_x = np.append(self.data_x, self.time)
            self.data_y = np.append(self.data_y, rpm)
            if len(self.data_x) > 100:
                self.data_x = self.data_x[-100:]
                self.data_y = self.data_y[-100:]

            dpg.set_value(self.line_series, [self.data_x, self.data_y])
            dpg.set_axis_limits(self.y_axis, 0, 150)
            dpg.fit_axis_data(self.x_axis)

        def set_command(self, rpm_cmd):
            if self.cmd_line:
                dpg.delete_item(self.cmd_line)
            self.cmd_line = dpg.add_inf_line_series([rpm_cmd], horizontal=True, parent=self.y_axis)


    def __init__(self):
        super().__init__("asserv_visualizer")

        # --- DearPyGui Setup ---
        dpg.create_context()
        dpg.create_viewport(title='Asserv Visualization', width=800, height=800)
        dpg.setup_dearpygui()

        # Create the window and graph on initialization
        with dpg.window(label="Asserv Window", width=800, height=800, no_collapse=True) as self.window:
            wheel_names = ["Front Left", "Front Right", "Back Right", "Back Left"]
            with dpg.group():
                self.wheelPlots = [self.WheelPlot(name) for name in wheel_names]

        dpg.show_viewport()

        # --- ROS Setup ---
        self.create_subscription(RPMs, '/robot/encoders', self.encoder_callback, 10)
        self.create_subscription(RPMs, '/robot/command', self.cmd_callback, 10)
        self.get_logger().info("Asserv visualizer node successfully launched")

    def encoder_callback(self, msg: RPMs):
        self.get_logger().info(
            f"received measured RPMS: {msg.front_left_rpm}, {msg.front_right_rpm}, {msg.back_right_rpm}, {msg.back_left_rpm}")

        self.wheelPlots[0].add_data(abs(msg.front_left_rpm))
        self.wheelPlots[1].add_data(abs(msg.front_right_rpm))
        self.wheelPlots[2].add_data(abs(msg.back_right_rpm))
        self.wheelPlots[3].add_data(abs(msg.back_left_rpm))

    def cmd_callback(self, msg: RPMs):
        self.wheelPlots[0].set_command(abs(msg.front_left_rpm))
        self.wheelPlots[1].set_command(abs(msg.front_right_rpm))
        self.wheelPlots[2].set_command(abs(msg.back_right_rpm))
        self.wheelPlots[3].set_command(abs(msg.back_left_rpm))

    def destroy_node(self):
        # Destroy the GUI context when the node is destroyed
        if dpg.is_dearpygui_running():
            dpg.destroy_context()
        super().destroy_node()


def main():
    rclpy.init()
    node = AsservVisualizer()

    # Custom render loop: alternate between checking ROS messages and drawing the GUI frame
    while dpg.is_dearpygui_running() and rclpy.ok():
        # timeout_sec=0 ensures it doesn't block the UI if there are no ROS messages
        rclpy.spin_once(node, timeout_sec=0)
        dpg.render_dearpygui_frame()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
