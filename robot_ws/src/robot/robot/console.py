import rclpy
from robot.steady_node import SteadyNode
from msgs.msg import RPMs, AsservParamChange
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from msgs.msg import RPMs, AsservParamChange, WheelSpeeds

DEFAULT_RPM = 150.0


class Console(SteadyNode):

    def __init__(self):
        super().__init__("console")
        self.last_cmd = "setrpm"

        self.pub_cmd = self.create_publisher(RPMs, "/robot/command", 10)
        self.pub_raw_cmd = self.create_publisher(WheelSpeeds, "/robot/raw_command", 10)
        self.pub_asserv_param = self.create_publisher(AsservParamChange, "robot/asserv_params", 10)
        self.pub_goal = self.create_publisher(Point, "/robot/automatic_goal", 10)
        self.pub_enable_auto = self.create_publisher(Bool, "/robot/enable_auto", 10)

        # This is technically bad since it blocks the main thread
        # But I had issues with the previous version with some desyncs
        # Console shouldn't really be receiving data anyways so i'd much
        # rather leave it like this. Feel free to change it though
        self.get_logger().info("Type help to get help on what commands exist and how to use them.")
        self.input_loop()

    # ---------------- UI ----------------
    def show_menu(self):
        print("\n==========================\n")
        #log += "Commands: setrpm | stop | setkp | setki | setkd | help \n"
        #log += f"Last : {self.last_cmd}\n"
        #log += "==========================\n"
        # self.get_logger().info(log)

    def input_loop(self):
        while rclpy.ok():
            self.show_menu()
            cmd = input("> ")

            if cmd == "":
                cmd = self.last_cmd
            else:
                self.last_cmd = cmd

            try:
                self.process(cmd)
            except:
                self.get_logger().error("Couldn't parse given command. Check if your arguments have the right type !")

    def process(self, cmd: str):


        split_cmd = cmd.strip().split(" ")
        command_name = split_cmd[0]

        def get_float_arg(arg_index: int):
            return float(split_cmd[arg_index])

        def get_int_arg(arg_index: int):
            return int(split_cmd[arg_index])

        if command_name == "manual":
            msg = Bool()
            msg.data = False
            self.pub_enable_auto.publish(msg)
            self.get_logger().info("Manual mode activated.")
        elif command_name == "automatic":
            if len(split_cmd) != 3:
                self.get_logger().error("The automatic command takes exactly 2 arguments (x and y).")
                return

            # automatic.py s'active
            msg_enable = Bool()
            msg_enable.data = True
            self.pub_enable_auto.publish(msg_enable)

            # On envoie la cible
            goal_msg = Point()
            goal_msg.x = get_float_arg(1)
            goal_msg.y = get_float_arg(2)
            goal_msg.z = 0.0
            self.pub_goal.publish(goal_msg)
        elif command_name == "setrpm":
            m = RPMs()
            if len(split_cmd) == 1:
                m.front_left_rpm = DEFAULT_RPM
                m.front_right_rpm = DEFAULT_RPM
                m.back_right_rpm = DEFAULT_RPM
                m.back_left_rpm = DEFAULT_RPM
            elif len(split_cmd) == 2:
                m.front_left_rpm = get_float_arg(1)
                m.front_right_rpm = get_float_arg(1)
                m.back_right_rpm = get_float_arg(1)
                m.back_left_rpm = get_float_arg(1)
            elif len(split_cmd) == 5:
                m.front_left_rpm = get_float_arg(1)
                m.front_right_rpm = get_float_arg(2)
                m.back_right_rpm = get_float_arg(3)
                m.back_left_rpm = get_float_arg(4)
            else:
                self.get_logger().error(f"The setrpm command either takes none, one or four arguments")

            self.pub_cmd.publish(m)
        elif command_name == "setpwm":
            m = WheelSpeeds()
            if len(split_cmd) == 2:
                m.front_left_wheel_speed = get_int_arg(1)
                m.front_right_wheel_speed = get_int_arg(1)
                m.back_right_wheel_speed = get_int_arg(1)
                m.back_left_wheel_speed = get_int_arg(1)
            elif len(split_cmd) == 5:
                m.front_left_wheel_speed = get_int_arg(1)
                m.front_right_wheel_speed = get_int_arg(2)
                m.back_right_wheel_speed = get_int_arg(3)
                m.back_left_wheel_speed = get_int_arg(4)
            else:
                self.get_logger().error(f"The setpwm command either takes one or four arguments")

            self.pub_raw_cmd.publish(m)
        elif command_name == "stop":
            msg = Bool()
            msg.data = False
            self.pub_enable_auto.publish(msg)

            m = WheelSpeeds()
            m.front_left_wheel_speed = 0
            m.front_right_wheel_speed = 0
            m.back_right_wheel_speed = 0
            m.back_left_wheel_speed = 0

            self.pub_raw_cmd.publish(m)

        elif command_name.startswith("set"):
            if len(split_cmd) != 2 and len(split_cmd) != 3:
                self.get_logger().error(f"set... commands takes 2 or 3 arguments")
                return
            
            param_id = ""
            if command_name == "setkp":
                param_id = "kp"
            elif command_name == "setki":
                param_id = "ki"
            elif command_name == "setkd":
                param_id = "kd"

            new_value = get_float_arg(2)

            def send_asser_param_message(wheel_id: str):
                m = AsservParamChange()
                m.wheel_id = wheel_id
                m.param_id = param_id
                if wheel_id != "frontleft" and wheel_id != "frontright"and wheel_id != "backright" and wheel_id != "backleft":
                    self.get_logger().error(f"Wrong wheel id used ! Use frontleft, frontright, backright or backleft")
                    return
                
                m.new_value = new_value
                
                self.pub_asserv_param.publish(m)

            if len(split_cmd) == 3:
                send_asser_param_message(split_cmd[1])
            else:
                send_asser_param_message("frontleft")
                send_asser_param_message("frontright")
                send_asser_param_message("backleft")
                send_asser_param_message("backright")
                

        elif command_name == "help":
            self.get_logger().info("""
Commands :
- automatic: Sends the robot to a target position. Can be used like so :
    >> automatic <x_target> <y_target>
- setrpm : Sets the RPMS of the wheels to the given speeds. Can be used like so :
    >> setrpm <front_left_wheel_rpm> <front_right_wheel_rpm> <back_right_wheel_rpm> <back_left_wheel_rpm> 
    >> setrpm <all_wheels_rpm>
    >> setrpm
    This last one is setting all wheels' rpms to 150
- setpwm : Sets the raw PWM sent to the wheels to a certain value between -255 and 255.
    >> setpwm <front_left_wheel_pwm> <front_right_wheel_pwm> <back_right_wheel_pwm> <back_left_wheel_pwm> 
    >> setpwm <all_wheels_pwm>
    >> setpwm
- stop: equivalent to setrpm 0
- setkp, setki, setkd: sets a coefficient for the servoing (asservissement) of a given wheel. Can be used like so :
    >> setkp frontleft 0.3
    >> setkd backright 2.3                     
""")
        else:
            self.get_logger().error(f"Unknown command \"{command_name}\"")
            return


def main():
    rclpy.init()
    rclpy.spin(Console())
    rclpy.shutdown()

main()
