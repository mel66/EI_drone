# command.py
behaviors = ['TakeOff', 'Land', 'Hover', 'MoveForward','MoveBackward','MoveRight', 'MoveLeft','TurnLeft', 'TurnRight','MoveUp','MoveDown']
commands = {
    'TakeOff': [(0, 'TakeOff')],
    'Land': [(0, 'Land')],
    'Hover': [(0, 'Hover')],
    'EmergencyStop': [(0, 'Hover'), (1.0, 'Land')],
    
    # Movement commands
    'MoveForward': [(0, 'MoveForward')],
    'MoveBackward': [(0, 'MoveBackward')],
    'MoveLeft': [(0, 'MoveLeft')],
    'MoveRight': [(0, 'MoveRight')],
    
    # Rotation commands
    'TurnLeft': [(0, 'TurnLeft')],
    'TurnRight': [(0, 'TurnRight')],
    
    # Vertical movement commands
    'MoveUp': [(0, 'MoveUp')],
    'MoveDown': [(0, 'MoveDown')]
}
SLOW_SPEED = 0.2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from behavior_interface.msg import Command,BehaviorStatus  # Assuming Command.msg is defined


import time
import heapq  # To manage a sorted queue of events

class CommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.behaviors = behaviors
        self.commands = commands
        self.active_behaviors = set()  # Track active behaviors

        # Queue to manage (activation time, behavior) events
        self.event_queue = []

        # Subscribe to the command topic
        self.command_subscription = self.create_subscription(Command, 'command', self.command_callback, 10)
        
        # Publisher to activate behaviors
        self.behavior_publisher = self.create_publisher(BehaviorStatus, 'behavior', 10)
        
        # Timer to periodically check and process the event queue
        self.timer = self.create_timer(0.05, self._on_time)

    def command_callback(self, msg):
        command_name = msg.command
        self.get_logger().info(f"Received command: {command_name}")

        if command_name in self.commands:
            self.execute_command(command_name)
        else:
            self.get_logger().warning(f"Unknown command: {command_name}")

    def execute_command(self, command_name):
        # Deactivate all behaviors
        self.deactivate_all_behaviors()

        # Get the current time
        start_time = time.time()

        # Schedule each behavior in the command with the appropriate delay
        for delay, behavior_name in self.commands[command_name]:
            activation_time = start_time + delay
            heapq.heappush(self.event_queue, (activation_time, behavior_name))

    def deactivate_all_behaviors(self):
        for behavior in self.behaviors:
            if behavior in self.active_behaviors:
                self.publish_behavior(behavior, activate=False)
                self.active_behaviors.remove(behavior)

    def publish_behavior(self, behavior_name, activate=True):
        behavior_msg = BehaviorStatus()
        behavior_msg.name = behavior_name
        behavior_msg.status = activate
        self.behavior_publisher.publish(behavior_msg)
        action = "Activating" if activate else "Deactivating"
        self.get_logger().info(f"{action} behavior: {behavior_name}")

    def _on_time(self):
        # Check if there are events ready to be processed in the event queue
        current_time = time.time()
        while self.event_queue and self.event_queue[0][0] <= current_time:
            _, behavior_name = heapq.heappop(self.event_queue)
            self.publish_behavior(behavior_name)
            self.active_behaviors.add(behavior_name)

def main(args=None):
    rclpy.init(args=args)
    command_node = CommandNode()
    rclpy.spin(command_node)
    command_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



    def deactivate_all_behaviors(self):
        for behavior in self.behaviors:
            if behavior in self.active_behaviors:
                self.publish_behavior(behavior, activate=False)
                self.active_behaviors.remove(behavior)

    def publish_behavior(self, behavior_name, activate=True):
        # Publish to behavior topic with BehaviorStatus message
        behavior_msg = BehaviorStatus()
        behavior_msg.name = behavior_name
        behavior_msg.status = activate
        self.behavior_publisher.publish(behavior_msg)
        
        action = "Activating" if activate else "Deactivating"
        self.get_logger().info(f"{action} behavior: {behavior_name}")

    def _on_time(self):
        # Check if there are events ready to be processed in the event queue
        current_time = time.time()
        while self.event_queue and self.event_queue[0][0] <= current_time:
            _, behavior_name = heapq.heappop(self.event_queue)
            self.publish_behavior(behavior_name)
            self.active_behaviors.add(behavior_name)

def main(args=None):
    rclpy.init(args=args)
    command_node = CommandNode()
    rclpy.spin(command_node)
    command_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
