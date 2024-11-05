#!/usr/bin/env python

# External imports
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CompressedImage
from example_interfaces.msg import Float32  # std_msgs.msg.Float32 is deprecated
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

# Local imports
from visual_processing import my_line_library as mll  # Assurez-vous que ce module est accessible


class Door(Node):
    def __init__(self):
        super().__init__("door")

        # Subscriber pour les images compressées
        self.img_sub = self.create_subscription(
            CompressedImage, "video_in", self.on_image, 1
        )
        self.bridge = CvBridge()
        self.previous_frame_line = None  # Stocke la ligne de l'image précédente

    def on_image(self, msg):
        # Convertir l'image ROS en format OpenCV
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info(f"Receiving video frame {frame.shape}, of type : {type(frame)}")
        
        # Convertir l'image en niveaux de gris
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Récupérer la ligne horizontale au milieu de l'image
        middle_row = gray.shape[0] // 2
        current_frame_line = gray[middle_row, :]
        
        # Comparer avec la ligne de l'image précédente si elle existe
        if self.previous_frame_line is not None:
            # Calculer le décalage entre les deux lignes
            shift = mll.local_shift(
                self.previous_frame_line, current_frame_line, max_shift=10, window_radius=5
            )
            self.get_logger().info(f"Calculated shift: {shift}")
        
        # Mettre à jour la ligne précédente pour la prochaine comparaison
        self.previous_frame_line = current_frame_line


def main(args=None):
    rclpy.init(args=args)
    door_node = Door()  # Crée une instance de la classe Door
    rclpy.spin(door_node)
    door_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()