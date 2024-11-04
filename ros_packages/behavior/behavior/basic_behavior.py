import rclpy
from rclpy.node import Node
from behavior_interface.msg import BehaviorStatus

class BaseBehavior(Node):
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        self.active = False

        # Subscription to the "behavior" topic to activate/deactivate the behavior
        self.create_subscription(
            BehaviorStatus,
            'behavior',
            self.status_callback,
            10
        )

        # Publisher to "behaviors_status" to send the current status
        self.status_publisher = self.create_publisher(
            BehaviorStatus,
            'behaviors_status',
            10
        )

    def status_callback(self, msg):
        # Activate or deactivate the behavior if the name matches
        if msg.name == self.name:
            self.active = msg.status
            self.on_status()
        elif msg.name == 'ping':
            # Send status in response to a ping
            self.send_status()

    def on_status(self):
        # Override this method in subclasses to define behavior-specific actions
        self.get_logger().info(f"{self.name} status set to {'active' if self.active else 'inactive'}")

    def send_status(self):
        status_msg = BehaviorStatus()
        status_msg.name = self.name
        status_msg.status = self.active
        self.status_publisher.publish(status_msg)


