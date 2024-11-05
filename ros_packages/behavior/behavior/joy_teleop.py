import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from behavior_interface.msg import Command

# Button and axis mappings
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
        self.deadman_button = BUTTON_RB  # Button index for the deadman button
        self.start_command = 'TakeOff'
        self.stop_command = 'EmergencyStop'
        
        # Create publisher to the command topic
        self.command_publisher = self.create_publisher(Command, 'command', 10)

        # Subscribe to the /joy topic
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Track state of the deadman button
        self.deadman_pressed = False

    def joy_callback(self, msg):
        # Check if the deadman button is pressed
        if msg.buttons[self.deadman_button] == 1:  # Button is pressed
            if not self.deadman_pressed:
                self.deadman_pressed = True
                self.send_command(self.start_command)

            # Process other commands only if the deadman button is pressed
            self.process_commands(msg)
        else:  # Deadman button is released
            if self.deadman_pressed:
                self.deadman_pressed = False
                self.send_command(self.stop_command)

    def send_command(self, command_str):
        # Publish a command to the command topic
        command_msg = Command()
        command_msg.command = command_str
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f"Published command: {command_str}")

    def process_commands(self, msg):
        # Define mappings for additional joystick commands
        command_mappings = {
            BUTTON_B: 'Hover',
            BUTTON_A: 'move_right',
            BUTTON_X: 'move_left',
            BUTTON_Y: 'move_up'
        }

        # Trigger commands based on button presses
        for button_index, command in command_mappings.items():
            if msg.buttons[button_index] == 1:  # Button is pressed
                self.send_command(command)

        # Handle joystick axes with a deadzone
        deadzone = 0.1

        # Forward/backward movement (Y-axis)
        if abs(msg.axes[AXIS_LEFT_VERTICAL]) > deadzone:
            if msg.axes[AXIS_LEFT_VERTICAL] > 0:
                self.send_command('move_forward')
            elif msg.axes[AXIS_LEFT_VERTICAL] < 0:
                self.send_command('move_backward')

        # Left/right movement (X-axis)
        if abs(msg.axes[AXIS_LEFT_HORIZONTAL]) > deadzone:
            if msg.axes[AXIS_LEFT_HORIZONTAL] > 0:
                self.send_command('turn_right')
            elif msg.axes[AXIS_LEFT_HORIZONTAL] < 0:
                self.send_command('turn_left')

def main(args=None):
    rclpy.init(args=args)
    joy_teleop = JoyTeleop()
    rclpy.spin(joy_teleop)
    joy_teleop.destroy_node()
    rclpy.shutdown()
