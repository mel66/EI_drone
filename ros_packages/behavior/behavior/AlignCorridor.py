import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class AlignCorridor(Node):
    def __init__(self):
        super().__init__('align_corridor')
        self.subscriber = self.create_subscription(Point, 'vanishing_point', self.align_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def align_callback(self, msg):
        twist_msg = Twist()
        image_center_x = 320  # Assuming a 640x480 image
        x_offset = msg.x - image_center_x

        # Adjust angular_z to align drone
        twist_msg.angular.z = -0.005 * x_offset  # Scale appropriately
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    align_corridor = AlignCorridor()
    rclpy.spin(align_corridor)
    align_corridor.destroy_node()
    rclpy.shutdown()



 <!--  Apply the vanishing point detection  -->
    <node pkg="visual_processing" exec="vp_node" name="vp_node" output="screen">
    <!--  <remap from="camera" to="/webcam/image_raw/compressed"/>  -->
    <remap from="video_in" to="/bebop/camera/image_raw/compressed"/>
    </node>
    <!--  Viewer for the images  -->
    <node pkg="rqt_image_view" exec="rqt_image_view" name="image_viewer" args="/webcam/image_raw"/>
    <node pkg ="rqt_gui" exec="rqt_gui"/>