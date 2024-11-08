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
from std_msgs.msg import Bool

# Local imports
from visual_processing import optical

class Door(Node):
    def __init__(self):
        super().__init__("door")

        # Subscriber pour les images compressées
        self.img_sub = self.create_subscription(
            CompressedImage, "video_in", self.on_image, 10
        )
        
        # Publisher pour publier les images de débogage
        self.pub_optical = self.create_publisher(CompressedImage, "video_out", 10)
        self.free_space_ahead = self.create_publisher(Bool, 'free_space_ahead', 10)

        self.bridge = CvBridge()
        self.previous_frame_line = None  # Stocke la ligne de l'image précédente
        self.previous_frame = None

        # Subscriber for drone speed
        self.speed_sub = self.create_subscription(
            Float32, "/speed", self._on_speed, 10
        )
        self.current_speed = 0.0
        self.gamma = 0.5 # Facteur de la caméra, à ajuster 
        self.MAX_DISTANCE = 1000


    def _on_speed(self, msg):
        self.current_speed = msg.data
        self.get_logger().info(f"Drone speed updated: {self.current_speed}")

    def on_image(self, msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Récupérer la ligne horizontale au milieu de l'image
        middle_row = gray.shape[0] // 2
        current_frame_line = gray[middle_row, :]
        

        max_shift = 40
        radius = 40

        # Comparer avec la ligne de l'image précédente si elle existe
        if self.previous_frame_line is not None:
            # Calculer le décalage entre les deux lignes
            shift = optical.local_shift(
                current_frame_line, self.previous_frame_line, max_shift=max_shift, window_radius=radius
            )
            
            # Calcul des distances pour chaque pixel en utilisant la formule d = gamma * v / f
            f = np.abs(shift)  # Flux optique pour chaque pixel
            distances = np.where(f != 0, (self.gamma * self.current_speed) / f, self.MAX_DISTANCE)  # Évite la division par zéro

            # Ajouter un padding pour aligner le shift avec les positions de l'image
            padding = max_shift + radius
            wall_door = optical.detect_door(current_frame_line, shift, padding)#La fonction distingue les mur de couleur uniforme des portes avec profondeur


            # Visualiser les courbes sur l'image
            img_display = self.previous_frame.copy()
            cv2.line(img_display, (0, middle_row), (img_display.shape[1], middle_row), (0, 0, 0), 1)

            # Dessiner les courbes d'intensité et le décalage sur l'image
            optical.draw_function(img_display, self.previous_frame_line, hmin=img_display.shape[0]//2, hmax=img_display.shape[0]-10, ymin=0, ymax=255, color=(0, 0, 255), thickness=1)
            optical.draw_function(img_display, current_frame_line, hmin=img_display.shape[0]//2, hmax=img_display.shape[0]-10, ymin=0, ymax=255, color=(0, 255, 0), thickness=1)
            optical.draw_function(img_display, wall_door, hmin=10, hmax=img_display.shape[0]//2-10, ymin=0, ymax=1, color=(255, 255, 0), thickness=1)
            optical.draw_function(img_display, distances, hmin=10, hmax=img_display.shape[0]//2-10, ymin=np.min(distances), ymax=np.max(distances), color=(255, 255, 0), thickness=1)


            # Rogner l'image pour retirer les bordures ajoutées par max_shift et radius
            img_display = img_display[:, max_shift + radius : - (max_shift + radius)]
            

            # Y a t il de l'espace en face
            # Définir une fenêtre centrée sur le milieu de `c`
            threshold_distance = 1.5  # Distance seuil pour considérer un espace libre

            window_size = 10
            start = max(0, len(wall_door) // 2 - window_size // 2)
            end = min(len(wall_door), len(wall_door) // 2 + window_size // 2)

            window_distances = distances[start:end]
            window_mur_door = wall_door[start:end]

            window = np.logical_and(window_distances < threshold_distance , window_mur_door == 1)
            # Calculer le pourcentage de valeurs égales à 1 dans la fenêtre
            percentage = np.sum(window) / len(window_distances)

            # Vérifier si le pourcentage dépasse 70 %
            free_space = Bool()
            free_space.data = bool(percentage >= 0.7)
            self.free_space_ahead.publish(free_space)

            # free_space = Bool()
            # if c[len(c)//2]:
            #     free_space.data = True
            # else: free_space.data = False
            # self.free_space_ahead.publish(free_space)

            # Convertir l'image annotée en message ROS compressé et publier
            outmsg = self.bridge.cv2_to_compressed_imgmsg(img_display.copy())
            self.pub_optical.publish(outmsg)

        # Mettre à jour la ligne précédente pour la prochaine comparaison
        self.previous_frame_line = current_frame_line
        self.previous_frame = frame
        # self.get_logger().info("New frame")


def main(args=None):
    rclpy.init(args=args)
    door_node = Door()
    rclpy.spin(door_node)
    door_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()