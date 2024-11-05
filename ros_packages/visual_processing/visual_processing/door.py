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
        
        # Convertir l'image en niveaux de gris
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Récupérer la ligne horizontale au milieu de l'image
        middle_row = gray.shape[0] // 2
        current_frame_line = gray[middle_row, :]

        # Comparer avec la ligne de l'image précédente si elle existe
        if self.previous_frame_line is not None:
            # Calculer le décalage entre les deux lignes
            shift = mll.local_shift(
                self.previous_frame_line, current_frame_line, max_shift=20, window_radius=9
            )

            # Appliquer un filtre médian pour lisser le signal du décalage
            shift_smoothed = median_filter(shift, size=21)

            # Ajouter un padding pour aligner le shift avec les positions de l'image
            padding = 20 + 9  # max_shift + window_radius
            shift_padded = np.pad(shift_smoothed, (padding, padding), mode='constant', constant_values=0)

            # Afficher les informations de décalage
            self.get_logger().info(f"Calculated smoothed shift: {shift_smoothed}")

            # Visualiser les courbes sur l'image
            img_display = frame.copy()
            cv2.line(img_display, (0, middle_row), (img_display.shape[1], middle_row), (0, 0, 0), 1)
            
            # Dessiner les courbes d'intensité et le décalage sur l'image
            mll.draw_function(img_display, self.previous_frame_line, hmin=50, hmax=img_display.shape[0]-50, ymin=0, ymax=255, color=(0, 0, 255), thickness=1)
            mll.draw_function(img_display, current_frame_line, hmin=50, hmax=img_display.shape[0]-50, ymin=0, ymax=255, color=(0, 255, 0), thickness=1)
            mll.draw_function(img_display, shift_padded, hmin=50, hmax=img_display.shape[0]-50, ymin=0, ymax=20, color=(255, 0, 0), thickness=1)
            
            # Afficher l'image avec les courbes
            cv2.imshow('Image with Shift Visualization', img_display)
            cv2.waitKey(1)  # Utiliser un petit délai pour que l'affichage soit en temps réel

        # Mettre à jour la ligne précédente pour la prochaine comparaison
        self.previous_frame_line = current_frame_line


def main(args=None):
    rclpy.init(args=args)
    door_node = Door()  # Crée une instance de la classe Door
    rclpy.spin(door_node)
    door_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  # Assurez-vous que toutes les fenêtres OpenCV sont fermées


if __name__ == "__main__":
    main()