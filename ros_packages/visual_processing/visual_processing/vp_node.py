#!/usr/bin/env python


# External imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
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
            CompressedImage, "/debug/vpimg/image_raw/compressed", 1
        )

        # Subscriber pour les images compressées
        self.img_sub = self.create_subscription(
            CompressedImage, "video_in", self.on_image, 1
        )
        self.bridge = CvBridge()

        self.horizontal_offset_pub = self.create_publisher(Float32, 'vp_offset', 10)
        self.angle_ratio_pub = self.create_publisher(Float32, 'vp_angle', 10)
        self.vp_detected_pub = self.vp(Bool, 'vp_detected', 10)

        self.min_length = 80
        self.up = np.array([[0],[1]])
        self.ceiling = 50
        self.show_vanish = 1
        self.show_lines = 1
        init_h_angle = 11
        init_v_angle = 11
        self.h_cos_tol = np.cos(init_h_angle*np.pi/180.0)
        self.v_cos_tol = np.cos(init_v_angle*np.pi/180.0)



    def cb_params(self, data):
        for p in data:
            name = p.name
            value = p.value
            setattr(self.vp_processor, name, value)
        return SetParametersResult(successful=True)

    def on_image(self, msg):

        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        
        # if self.debug_pub.get_subscription_count() > 0:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lines = self.lsd.detect(gray)[0]
        lines = mll.from_lsd(lines)

        line_lengths = np.linalg.norm(lines[:, 2:4] - lines[:, 0:2], axis=1)
        lines = lines[line_lengths > self.min_length]

        # if self.show_lines:
        #     mll.draw_lines(frame, lines, (20, 100, 100), 1)

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

        # if self.show_lines:
        #     mll.draw_lines(frame, lines, (50, 255, 255), 1)

        ab = lines[:, 4:6] * np.array([1, -1])  # Vectors perpendicular to lines
        if ab.shape[0]>1: #Si il y a au moins 2 lignes SINON on n'a pas de point de fuite
            cluster_1_lines, cluster_2_lines = mll.vanishing_lines(lines, ab)

            cluster_1_lines = cluster_1_lines.reshape(-1,8)
            cluster_2_lines = cluster_2_lines.reshape(-1,8)
            

            mean1 = np.mean(cluster_1_lines,axis=0)
            mean2 = np.mean(cluster_2_lines,axis=0)
            
            x1 = -mean1[6]/mean1[4]
            x2 = -mean2[6]/mean2[4]
            if x1<x2:
                left = mean1
                right = mean2
            else:
                left = mean2
                right = mean1

            # Afficher les lignes des deux clusters avec des couleurs différentes
            if self.show_lines:
                mll.draw_lines(frame, np.array([right]), (0, 0, 255), 1)  # droite en vert
                mll.draw_lines(frame, np.array([left]), (0, 255, 0), 1)  # gauche en rouge

            lines = np.stack((mean1,mean2))
            intersections = mll.intersections(lines)
            x = int(intersections[0][0])
            y = int(intersections[0][1])
            vanish = [x,y]
            if 0 <= x <= frame.shape[1] and 0 <= y <= frame.shape[0]:
                vp_detected_msg= Bool()
                vp_detected_msg.data = True
                self.vp_detected_pub.publish(vp_detected_msg)
                if self.show_vanish :
                    cv2.circle(frame, vanish, 8, (255, 0, 0), 3) #vanishing point en bleu
                horizontal_offset = mll.horizontal_misplacement(frame,vanish)
                angle_ratio = mll.angle_indicator(left[4:6],right[4:6])
                # Publish horizontal offset and angle ratio
                horizontal_msg = Float32()
                horizontal_msg.data = float(horizontal_offset)
                self.horizontal_offset_pub.publish(horizontal_msg)

                angle_msg = Float32()
                angle_msg.data = float(angle_ratio)
                self.angle_ratio_pub.publish(angle_msg)
                # print(f"Leften side of frame to vanishing pt : {mll.horizontal_misplacement(frame,vanish)}")
                # print(f"Ratio angles : {mll.angle_indicator(left[4:6],right[4:6]) - 1}") 
            else:
                vp_detected_msg= Bool()
                vp_detected_msg.data = False
                self.vp_detected_pub.publish(vp_detected_msg)
        else :
            vp_detected_msg= Bool()
            vp_detected_msg.data = False
            self.vp_detected_pub.publish(vp_detected_msg)

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
