#!/usr/bin/env python


# External imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32 # example_interfaces.msg.Float32 is deprecated
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Bool



# Local imports
from visual_processing import my_line_library as mll



class VPNode(Node):
    def __init__(self):
        super().__init__("vp_node")
        self.lsd = cv2.createLineSegmentDetector(0)
        # Publisher d'image de debug
        self.debug_pub = self.create_publisher(
            CompressedImage, "video_out", 1
        )

        # Subscriber pour les images compressées
        self.img_sub = self.create_subscription(
            CompressedImage, "video_in", self.on_image, 1
        )
        self.bridge = CvBridge()

        self.horizontal_offset_pub = self.create_publisher(Float32, 'vp_offset', 10)
        self.angle_ratio_pub = self.create_publisher(Float32, 'vp_angle', 10)
        self.vp = self.create_publisher(Bool, 'vp_detected', 10)


        self.min_length = 80
        self.up = np.array([[0],[1]])
        self.ceiling = 50
        self.show_vanish = 1
        self.show_lines = 1
        init_h_angle = 45
        init_v_angle = 11
        self.h_cos_tol = np.cos(init_h_angle*np.pi/180.0)
        self.v_cos_tol = np.cos(init_v_angle*np.pi/180.0)

    def on_image(self, msg):

        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lines = self.lsd.detect(gray)[0]
        lines = mll.from_lsd(lines)

        line_lengths = np.linalg.norm(lines[:, 2:4] - lines[:, 0:2], axis=1)
        lines = lines[line_lengths > self.min_length]


        # Filter lines above the ceiling
        lines = lines[np.logical_and(lines[:, 1] > self.ceiling, lines[:, 3] > self.ceiling)]

        # Filter out lines that are too horizontal
        ab = lines[:, 4:6] * np.array([1, -1])  # Adjust frame direction
        scal = np.dot(ab, self.up.flatten())
        lines = lines[np.abs(scal) < self.h_cos_tol]
        ab = ab[np.abs(scal) < self.h_cos_tol]
        # Filter out lines that are too vertical
        scal = np.dot(ab, np.array([1, 0]))
        lines = lines[np.abs(scal) < self.v_cos_tol]


        ab = lines[:, 4:6] * np.array([1, -1])  # Vectors perpendicular to lines
        if ab.shape[0]>1: #Si il y a au moins 2 lignes SINON on n'a pas de point de fuite
            lines = mll.vanishing_lines(lines, ab)
            if lines:
                left, right = lines
                if self.show_lines:
                    mll.draw_lines(frame, np.array([right]), (0, 0, 255), 1)  # droite en vert
                    mll.draw_lines(frame, np.array([left]), (0, 255, 0), 1)  # gauche en rouge
                vanish = mll.vp_point(left, right)
                if 0 <= vanish[0] <= frame.shape[1] and 0 <= vanish[1] <= frame.shape[0]: #Si le point de fuite existe
                    vp_msg = Bool()
                    vp_msg.data = True
                    self.vp.publish(vp_msg)

                    if self.show_vanish :#On dessine le point
                        cv2.circle(frame, vanish, 8, (255, 0, 0), 3) #vanishing point en bleu

                    ###On calcule les indices###
                    horizontal_offset = mll.horizontal_misplacement(frame,vanish)
                    angle_ratio = mll.angle_indicator(left[4:6],right[4:6])

                    horizontal_msg = Float32()
                    horizontal_msg.data = float(horizontal_offset)
                    self.horizontal_offset_pub.publish(horizontal_msg)

                    angle_msg = Float32()
                    angle_msg.data = float(angle_ratio)
                    self.angle_ratio_pub.publish(angle_msg)
                else:
                    vp_msg = Bool()
                    vp_msg.data = False
                    self.vp.publish(vp_msg)
                    
            else:
                vp_msg = Bool()
                vp_msg.data = False
                self.vp.publish(vp_msg)  
        else :
            vp_msg = Bool()
            vp_msg.data = False
            self.vp.publish(vp_msg)
            
        outmsg = self.bridge.cv2_to_compressed_imgmsg(frame.copy())
        self.debug_pub.publish(outmsg)


def main(args=None):
    rclpy.init(args=args)
    vp_node = VPNode()
    rclpy.spin(vp_node)
    vp_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
