import rclpy
from .basic_behavior import BaseBehavior
from std_msgs.msg import Float32, Bool
import time
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



  

class UTurn(BaseBehavior):
    def __init__(self, rotation_duration=2.0, rotation_speed=SLOW_SPEED):
        super().__init__('UTurn')
        
        # Parameters for rotation
        self.rotation_duration = rotation_duration  # Duration in seconds
        self.rotation_speed = rotation_speed        # Angular speed (radians per second)
        
        # Publisher for `angular_z`
        self.angular_z_pub = self.create_publisher(Float32, 'angular_z', 10)
        self.active = False

    def on_status_on(self):
        self.active = True
        self.perform_u_turn()

    def perform_u_turn(self):
        # Start rotating
        end_time = time.time() + self.rotation_duration
        while time.time() < end_time and self.active:
            self.angular_z_pub.publish(Float32(data=self.rotation_speed))
            time.sleep(0.1)
        
        # Stop rotation
        self.angular_z_pub.publish(Float32(data=0.0))
        self.active = False


def UTurnmain(args=None):
    rclpy.init(args=args)
    u_turn = UTurn()
    rclpy.spin(u_turn)
    u_turn.destroy_node()
    rclpy.shutdown()



class Slide(BaseBehavior):
    def __init__(self, slide_direction, orientation_duration=1.0, slide_duration=2.0, slide_speed=SLOW_SPEED):
        super().__init__(f'Slide{slide_direction}')
        
        # Parameters for sliding and orientation
        self.orientation_duration = orientation_duration  # Time to face lateral wall
        self.slide_duration = slide_duration              # Time to slide
        self.slide_speed = slide_speed                    # Speed during sliding
        self.slide_direction = slide_direction            # 'left' or 'right'

        # Publishers for `angular_z` and `linear_y`
        self.angular_z_pub = self.create_publisher(Float32, 'angular_z', 10)
        self.linear_y_pub = self.create_publisher(Float32, 'linear_y', 10)
        self.active = False

    def on_status_on(self):
        self.active = True
        self.perform_slide()

    def perform_slide(self):
        # Step 1: Change orientation to face lateral wall
        rotation_speed = SLOW_SPEED if self.slide_direction == 'Left' else -SLOW_SPEED
        end_time = time.time() + self.orientation_duration
        while time.time() < end_time and self.active:
            self.angular_z_pub.publish(Float32(data=rotation_speed))
            time.sleep(0.1)
        self.angular_z_pub.publish(Float32(data=0.0))

        # Step 2: Slide along the wall
       # Step 2: Slide movement
        slide_speed = self.slide_speed if self.slide_direction == 'Left' else -self.slide_speed
        end_time = time.time() + self.slide_duration
        while time.time() < end_time and self.active:
            self.linear_y_pub.publish(Float32(data=slide_speed))
            time.sleep(0.1)
        
        # Stop sliding and deactivate behavior
        self.linear_y_pub.publish(Float32(data=0.0))
        self.active = False



# Child class for sliding left
class SlideLeft(Slide):
    def __init__(self, orientation_duration=1.0, slide_duration=2.0, slide_speed=SLOW_SPEED):
        super().__init__(slide_direction='Left', orientation_duration=orientation_duration, slide_duration=slide_duration, slide_speed=slide_speed)

# Child class for sliding right
class SlideRight(Slide):
    def __init__(self, orientation_duration=1.0, slide_duration=2.0, slide_speed=SLOW_SPEED):
        super().__init__(slide_direction='Right', orientation_duration=orientation_duration, slide_duration=slide_duration, slide_speed=slide_speed)

def SlideLeftmain(args=None, direction='Left'):
    rclpy.init(args=args)
    slideLeft = Slide(slide_direction=direction)
    rclpy.spin(slideLeft)
    slideLeft.destroy_node()
    rclpy.shutdown()

def SlideRightmain(args=None, direction='Right'):
    rclpy.init(args=args)
    slideRight = Slide(slide_direction=direction)
    rclpy.spin(slideRight)
    slideRight.destroy_node()
    rclpy.shutdown()



class DoorCrossing(BaseBehavior):
    def __init__(self, direction, detection_timeout=3.0):
        super().__init__(f'DoorCrossing{direction}')
        
        # Door crossing parameters
        self.direction = direction                   # 'left' or 'right'
        self.detection_timeout = detection_timeout   # Maximum time to search for door
        self.cross_speed = SLOW_SPEED                     # Speed to cross door

        # Publisher for `linear_y`
        self.linear_y_pub = self.create_publisher(Float32, 'linear_y', 10)

        # Subscription for optical flow-based door detection
        self.door_detected_sub = self.create_subscription(Bool, f'door_{direction}_detected', self.door_callback, 10)
        self.door_detected = False

    def door_callback(self, msg):
        # Update the door detection status
        self.door_detected = msg.data

    def perform_door_crossing(self):
        start_time = time.time()
        while not self.door_detected and (time.time() - start_time) < self.detection_timeout and self.active:
            time.sleep(0.1)

        if self.door_detected and self.active:
            # Move laterally to cross the door
            self.linear_y_pub.publish(Float32(data=self.cross_speed if self.direction == 'Left' else -self.cross_speed))
            time.sleep(1.5)  # Cross door duration
            self.linear_y_pub.publish(Float32(data=0.0))  # Stop after crossing
        else:
            print("Door not detected within timeout.")


# Child class for sliding left
class DoorCrossingLeft(DoorCrossing):
    def __init__(self, direction, detection_timeout=3.0):
        super().__init__(slide_direction='Left', detection_timeout=3.0)

# Child class for sliding right
class DoorCrossingRight(DoorCrossing):
    def __init__(self, direction, detection_timeout=3.0):
        super().__init__(slide_direction='Right', detection_timeout=3.0)

def DoorCrossingLeftmain(args=None, direction='Left'):
    rclpy.init(args=args)
    CrossLeft = Slide(slide_direction=direction)
    rclpy.spin(CrossLeft)
    CrossLeft.destroy_node()
    rclpy.shutdown()

def DoorCrossingRightmain(args=None, direction='Right'):
    rclpy.init(args=args)
    CrossRight = Slide(slide_direction=direction)
    rclpy.spin(CrossRight)
    CrossRight.destroy_node()
    rclpy.shutdown()
