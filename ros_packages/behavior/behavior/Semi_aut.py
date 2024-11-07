import rclpy
from .basic_behavior import BaseBehavior
from std_msgs.msg import Float32, Bool
import math
from .command import SLOW_SPEED


class AlignCorridor(BaseBehavior):
    def __init__(self):
        super().__init__('AlignCorridor')
        
        # Abonnements au topic `vp_offset` et à l'état de détection de point de fuite
        self.offset_sub = self.create_subscription(Float32, 'vp_offset', self.offset_callback, 10)
        self.vp_detected_sub = self.create_subscription(Bool, 'vp_detected', self.vp_detected_callback, 10)
        
        # Publisher pour `angular_z`
        self.AlignCorridor_publisher = self.create_publisher(Float32, 'angular_z', 10)
        
        # Variables pour stocker l'état de détection et l'offset
        self.x_offset = 0.0
        self.vp_detected = False

    def offset_callback(self, msg):
        # Met à jour l'offset du point de fuite
        self.x_offset = msg.data
        self.update_alignment()

    def vp_detected_callback(self, msg):
        # Met à jour l'état de détection du point de fuite
        self.vp_detected = msg.data
        self.update_alignment()

    def update_alignment(self):
        # Vérifie si le point de fuite est détecté
        if self.vp_detected and self.active:
            angular_z_value = -SLOW_SPEED*self.x_offset

            self.AlignCorridor_publisher.publish(Float32(data=angular_z_value))
        else:
            # Si pas de point de fuite, ne publie rien (ou publiez un message avec `angular_z = 0` si besoin)
            pass


def AlignCorridormain(args=None):
    rclpy.init(args=args)
    align_corridor = AlignCorridor()
    rclpy.spin(align_corridor)
    align_corridor.destroy_node()
    rclpy.shutdown()


class CenterCorridor(BaseBehavior):
    def __init__(self):
        super().__init__('CenterCorridor')
        
        # Abonnements au topic `vp_angle` et à l'état de détection de point de fuite
        self.angle_sub = self.create_subscription(Float32, 'vp_angle', self.angle_callback, 10)
        self.vp_detected_sub = self.create_subscription(Bool, 'vp_detected', self.vp_detected_callback, 10)
        
        # Publisher pour `linear_y`
        self.Center_publisher = self.create_publisher(Float32, 'linear_y', 10)
        
        # Variables pour stocker l'état de détection et le ratio d'angle
        self.angle_ratio = 0.0
        self.vp_detected = False

    def angle_callback(self, msg):
        # Met à jour le ratio d'angle du point de fuite
        self.angle_ratio = msg.data
        self.update_centering()

    def vp_detected_callback(self, msg):
        # Met à jour l'état de détection du point de fuite
        self.vp_detected = msg.data
        self.update_centering()

    def update_centering(self):
        # Vérifie si le point de fuite est détecté
        if self.vp_detected and self.active:
            linear_y_value = SLOW_SPEED*self.angle_ratio*2
            self.Center_publisher.publish(Float32(data=linear_y_value))
        else:
            # Si pas de point de fuite, ne publie rien (ou publiez un message avec `linear_y = 0` si besoin)
            pass


def CenterCorridormain(args=None):
    rclpy.init(args=args)
    center_corridor = CenterCorridor()
    rclpy.spin(center_corridor)
    center_corridor.destroy_node()
    rclpy.shutdown()




class MoveForwardVp(BaseBehavior):
    def __init__(self):
        super().__init__('MoveForwardVp')
        
        # Subscribe to the vanishing point detection topic
        self.vp_detected_sub = self.create_subscription(Bool, 'vp_detected', self.vp_detected_callback, 10)
        
        # Publisher for `linear_x` to control forward speed
        self.MoveFoward_publisher = self.create_publisher(Float32, 'linear_x', 10)
        
        # Variable to store detection status of vanishing point
        self.vp_detected = False

    def vp_detected_callback(self, msg):
        # Update the detection status of the vanishing point
        self.vp_detected = msg.data
        self.update_forward_motion()

    def update_forward_motion(self):
        # Check if the vanishing point is detected
        if self.vp_detected and self.active:
            # Move forward with a constant speed when vanishing point is detected
            forward_speed_value = SLOW_SPEED
            self.MoveFoward_publisher.publish(Float32(data=forward_speed_value))
        else:
            # Stop forward motion if no vanishing point is detected
            self.MoveFoward_publisher.publish(Float32(data=0.0))


def MoveForwardVpmain(args=None):
    rclpy.init(args=args)
    move_forward_vp = MoveForwardVp()
    rclpy.spin(move_forward_vp)
    move_forward_vp.destroy_node()
    rclpy.shutdown()
