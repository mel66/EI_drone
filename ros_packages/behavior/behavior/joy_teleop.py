import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from behavior_interface.msg import Command


BUTTON_A               =  0
BUTTON_B               =  1
BUTTON_X               =  2
BUTTON_Y               =  3
BUTTON_LB              =  4
BUTTON_RB              =  5
BUTTON_BACK            =  6
BUTTON_SELECT          =  7
BUTTON_LOGITECH        =  8
BUTTON_CLICK_LEFT_PAD  =  9
BUTTON_CLICK_RIGHT_PAD = 10

AXIS_LEFT_HORIZONTAL   =  0 # Left  joystick
AXIS_LEFT_VERTICAL     =  1 # Left  joystick
AXIS_LT                =  2 # Left  progressive button
AXIS_RIGHT_HORIZONTAL  =  3 # Right joystick
AXIS_RIGHT_VERTICAL    =  4 # Right joystick
AXIS_RT                =  5 # Right progressive button
AXIS_CROSS_HORIZONTAL  =  6 # Cross
AXIS_CROSS_VERTICAL    =  7 # Cross

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.command_publisher = self.create_publisher(Command, 'command', 10)

        self.deadman_button = 5  # Replace with your joystick's deadman button index
        self.start_command = 'take_off'
        self.stop_command = 'stop_and_land'

        self.deadman_pressed = False

    def joy_callback(self, msg):
        # Check if the deadman button is pressed
        if msg.buttons[self.deadman_button] == 1:  # Button is pressed
            if not self.deadman_pressed:
                self.deadman_pressed = True
                self.publish_command(self.start_command)
            self.process_commands(msg)
        else:  # Deadman button is released
            if self.deadman_pressed:
                self.deadman_pressed = False
                self.publish_command(self.stop_command)

    def publish_command(self, command_str):
        command_msg = Command()
        command_msg.command = command_str
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f"Published command: {command_str}")

    def process_commands(self, msg):
        # Example of processing other joystick buttons
        if msg.buttons[0] == 1:  # Button index for a command
            self.publish_command('fake_command_1')
        if msg.buttons[1] == 1:  # Another command
            self.publish_command('fake_command_2')
        # Add more commands as needed

def main(args=None):
    rclpy.init(args=args)
    joy_teleop = JoyTeleop()
    rclpy.spin(joy_teleop)
    joy_teleop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

