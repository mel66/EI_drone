import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy 
from std_msgs.msg import Empty

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
        self.hover_pub = self.create_publisher(Empty, '/bebop/hover', 10)

        self.deadzone = 0.75
        # Track the previous state of each axis to detect direction changes
        self.axis_states = {
            'MoveForwardBackward': 0,
            'MoveLeftRight': 0,
            'TurnLeftRight': 0,
            'MoveUpDown': 0
        }
        # Subscribe to the /joy topic
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Track state of the deadman button
        self.deadman_pressed = False
        self.last_button_states = {}  # Track the previous state of each button

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

    def hover_command(self):
        # Publish an empty message to stop the current behavior
        self.hover_pub.publish(Empty())
        self.get_logger().info("Published hover command")
    

    def process_commands(self, msg):
        # Define mappings for additional joystick commands
        command_mappings = {
            BUTTON_B: 'Hover',
            BUTTON_Y: 'MoveUp',
            BUTTON_A: 'MoveDown',
        }

        for button_index, command in command_mappings.items():
            current_state = msg.buttons[button_index]

            # Check if the button state has changed (pressed or released)
            if button_index not in self.last_button_states or self.last_button_states[button_index] != current_state:
                # Update the last known state
                self.last_button_states[button_index] = current_state

                # If the button is pressed, send the command
                if current_state == 1:
                    self.send_command(command)
                # If the button is released, stop the command (if needed)
                elif current_state == 0:
                    pass



        # Deadzone for minimal joystick movement
       
        if abs(msg.axes[AXIS_LEFT_VERTICAL]) > self.deadzone:
            if msg.axes[AXIS_LEFT_VERTICAL] > 0 and self.axis_states['MoveForwardBackward'] <= 0:
                self.send_command('MoveForward')
                self.axis_states['MoveForwardBackward'] = 1
            elif msg.axes[AXIS_LEFT_VERTICAL] < 0 and self.axis_states['MoveForwardBackward'] >= 0:
                self.get_logger().info(f"je suis la {self.axis_states['MoveForwardBackward']}")
                self.send_command('MoveBackward')
                self.axis_states['MoveForwardBackward'] = -1
        elif self.axis_states['MoveForwardBackward'] != 0:
            self.axis_states['MoveForwardBackward'] = 0
        
       
        # Left/right movement (X-axis) - Left joystick

        if abs(msg.axes[AXIS_LEFT_HORIZONTAL]) > self.deadzone:
            if msg.axes[AXIS_LEFT_HORIZONTAL] > 0 and self.axis_states['MoveLeftRight'] <= 0:
                self.send_command('MoveRight')
                self.axis_states['MoveLeftRight'] = 1
            elif msg.axes[AXIS_LEFT_HORIZONTAL] < 0 and self.axis_states['MoveLeftRight'] >= 0:
                self.send_command('MoveLeft')
                self.axis_states['MoveLeftRight'] = -1
        elif self.axis_states['MoveLeftRight'] != 0:
            self.axis_states['MoveLeftRight'] = 0

        # Turning (X-axis) - Right joystick
     
        if abs(msg.axes[AXIS_RIGHT_HORIZONTAL]) > self.deadzone:
            if msg.axes[AXIS_RIGHT_HORIZONTAL] > 0 and self.axis_states['TurnLeftRight'] <= 0:
                self.send_command('TurnLeft')
                self.axis_states['TurnLeftRight'] = 1
            elif msg.axes[AXIS_RIGHT_HORIZONTAL] < 0 and self.axis_states['TurnLeftRight'] >= 0:
                self.send_command('TurnRight')
                self.axis_states['TurnLeftRight'] = -1
        elif self.axis_states['TurnLeftRight'] != 0:
            self.axis_states['TurnLeftRight'] = 0


    
       
def main(args=None):
    rclpy.init(args=args)
    joy_teleop = JoyTeleop()
    rclpy.spin(joy_teleop)
    joy_teleop.destroy_node()
    rclpy.shutdown()


