#!/usr/bin/env python

# External imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from example_interfaces.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.ndimage import median_filter
from std_msgs.msg import String

# Local imports
from visual_processing import my_line_library as mll  # Assurez-vous que ce module est accessible
from visual_processing import optical

class Door(Node):
    def __init__(self):
        super().__init__("door")

        # Subscriber pour les images compressées
        self.img_sub = self.create_subscription(
            CompressedImage, "video_in", self.on_image, 10
        )
        
        # Publisher pour publier les images de débogage
        self.pub_optical = self.create_publisher(CompressedImage, "video_out/compressed", 10)
        
        self.bridge = CvBridge()
        self.previous_frame_line = None  # Stocke la ligne de l'image précédente

        # Subscriber for drone speed
        self.speed_sub = self.create_subscription(
            Float32, "/speed", self._on_speed, 10
        )
        self.current_speed = 0.0

    def _on_speed(self, msg):
        self.current_speed = msg.data
        self.get_logger().info(f"Drone speed updated: {self.current_speed}")

    def on_image(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Récupérer la ligne horizontale au milieu de l'image
        middle_row = gray.shape[0] // 2
        current_frame_line = gray[middle_row, :]

        max_shift = 25
        radius = 40

        # Comparer avec la ligne de l'image précédente si elle existe
        if self.previous_frame_line is not None:
            # Calculer le décalage entre les deux lignes
            shift = optical.local_shift(
                current_frame_line, self.previous_frame_line, max_shift=max_shift, window_radius=radius
            )
            
            # Appliquer un filtre médian pour lisser le signal du décalage
            shift_smoothed = median_filter(shift, size=40)

    


            # Ajouter un padding pour aligner le shift avec les positions de l'image
            padding = max_shift + radius
            shift_padded = np.pad(shift_smoothed, (padding, padding), mode='constant', constant_values=0)

            c = optical.detect_door(self.previous_frame_line, current_frame_line, shift_padded, padding)


            # Visualiser les courbes sur l'image
            img_display = frame.copy()
            cv2.line(img_display, (0, middle_row), (img_display.shape[1], middle_row), (0, 0, 0), 1)

            # Dessiner les courbes d'intensité et le décalage sur l'image
            optical.draw_function(img_display, self.previous_frame_line, hmin=img_display.shape[0]//2, hmax=img_display.shape[0]-10, ymin=0, ymax=255, color=(0, 0, 255), thickness=1)
            optical.draw_function(img_display, current_frame_line, hmin=img_display.shape[0]//2, hmax=img_display.shape[0]-10, ymin=0, ymax=255, color=(0, 255, 0), thickness=1)
            optical.draw_function(img_display, shift_padded, hmin=10, hmax=img_display.shape[0]//2-10, ymin=0, ymax=max_shift, color=(255, 0, 0), thickness=1)
            optical.draw_function(img_display, c, hmin=10, hmax=img_display.shape[0]//2-10, ymin=0, ymax=1, color=(255, 255, 0), thickness=1)

            # Rogner l'image pour retirer les bordures ajoutées par max_shift et radius
            img_display = img_display[:, max_shift + radius : - (max_shift + radius)]

            # Convertir l'image annotée en message ROS compressé et publier
            outmsg = self.bridge.cv2_to_compressed_imgmsg(img_display.copy())
            self.pub_optical.publish(outmsg)

        # Mettre à jour la ligne précédente pour la prochaine comparaison
        self.previous_frame_line = current_frame_line
        self.get_logger().info("New frame")


def main(args=None):
    rclpy.init(args=args)
    door_node = Door()
    rclpy.spin(door_node)
    door_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()