import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from behavior_interface.msg import Command


BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_SELECT = 7
BUTTON_LOGITECH = 8
BUTTON_CLICK_LEFT_PAD = 9
BUTTON_CLICK_RIGHT_PAD = 10

AXIS_LEFT_HORIZONTAL = 0
AXIS_LEFT_VERTICAL = 1
AXIS_LT = 2
AXIS_RIGHT_HORIZONTAL = 3
AXIS_RIGHT_VERTICAL = 4
AXIS_RT = 5
AXIS_CROSS_HORIZONTAL = 6
AXIS_CROSS_VERTICAL = 7

BUTTON_DEADMAN = BUTTON_LB
AXIS_LINEAR = AXIS_LEFT_VERTICAL
AXIS_ANGULAR = AXIS_LEFT_HORIZONTAL


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # Parameters for the joystick mappings
        self.deadman_button = BUTTON_RB  # Button index for the deadman button (example index)
        self.start_command = 'take_off'
        self.stop_command = 'stop_and_land'
        
        # Create publisher to the command topic
        self.command_publisher = self.create_publisher(Command, 'command', 10)

        # Subscribe to the /joy topic
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.deadman_pressed = False

    def joy_callback(self, msg):
        print(msg.buttons[self.deadman_button])
        # Check if the deadman button is pressed
        if msg.buttons[self.deadman_button] == 1:  # Button is pressed
            if not self.deadman_pressed:
                self.deadman_pressed = True
                self.send_command(self.start_command)

            # Process other commands only if the deadman is pressed
            self.process_commands(msg)
        else:  # Deadman button is released
            if self.deadman_pressed:
                self.deadman_pressed = False
                self.send_command(self.stop_command)

    def send_command(self, command_str):
        # Publish a command to the command topic
        command_msg = Command ()
        command_msg.command = command_str
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f"Published command: {command_str}")

    def process_commands(self, msg):
        # Example command mappings to buttons for fake commands
        fake_command_mappings = {
            BUTTON_A: 'fake_command_1',  # Button 0
            BUTTON_B: 'fake_command_2',  # Button 1
            BUTTON_X: 'fake_command_3'   # Button 2
        }

        # Check each button for fake commands
        for button_index, command in fake_command_mappings.items():
            if msg.buttons[button_index] == 1:  # Button is pressed
                self.send_command(command)

        # Handle joystick axis with a deadzone
        deadzone = 0.1
        if abs(msg.axes[1]) > deadzone:  # Y-axis (e.g., forward/backward)
            if msg.axes[1] > 0:
                self.send_command('move_forward')
            elif msg.axes[1] < 0:
                self.send_command('move_backward')

def main(args=None):
    rclpy.init(args=args)

    joy_teleop = JoyTeleop()
    rclpy.spin(joy_teleop)

    joy_teleop.destroy_node()
    rclpy.shutdown()



