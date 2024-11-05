import cv2
import numpy as np
import my_line_library as mll  # Assurez-vous que ce module est accessible
from scipy.ndimage import median_filter

# Charger les deux images
frame1 = cv2.imread("frame_t1.jpg")  # Remplacez par le chemin de votre première image
frame2 = cv2.imread("frame_t2.jpg")  # Remplacez par le chemin de votre deuxième image

# Vérifier que les images ont été correctement chargées
if frame1 is None or frame2 is None:
    raise FileNotFoundError("Vérifiez les chemins des images et assurez-vous que les fichiers existent.")

# Convertir les images en niveaux de gris
gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

# Extraire les lignes médianes des images
middle_row1 = gray1.shape[0] // 2
line1 = gray1[middle_row1, :]
line2 = gray2[middle_row1, :]  # On suppose que les images sont de même taille

# Définir les paramètres pour la fonction de décalage
max_shift = 20  # Décalage maximum à tester
window_radius = 9  # Rayon de la fenêtre locale

# Calculer le décalage entre les deux lignes
shift = mll.local_shift(line1, line2, max_shift=max_shift, window_radius=window_radius)

# Appliquer un filtre médian au décalage pour réduire le bruit
filtered_shift = median_filter(shift, size=21)  # Ajustez la taille de la fenêtre selon vos besoins

# Ajouter un padding pour aligner le décalage avec les positions de la ligne sur l'image
padding = max_shift + window_radius
shift_padded = np.pad(filtered_shift, (padding, padding), mode='constant', constant_values=0)

# Créer une copie de l'image pour le dessin
img_display = frame1.copy()

# Dessiner la ligne médiane
cv2.line(img_display, (0, middle_row1), (img_display.shape[1], middle_row1), (0, 0, 0), 1)

# Dessiner les courbes d'intensité et le décalage lissé sur l'image
mll.draw_function(img_display, line1, hmin=50, hmax=img_display.shape[0]-50, ymin=0, ymax=255, color=(0, 0, 255), thickness=1)
mll.draw_function(img_display, line2, hmin=50, hmax=img_display.shape[0]-50, ymin=0, ymax=255, color=(0, 255, 0), thickness=1)
mll.draw_function(img_display, shift_padded, hmin=50, hmax=img_display.shape[0]-50, ymin=0, ymax=max_shift, color=(255, 0, 0), thickness=1)

# Afficher l'image avec les courbes et le décalage
cv2.imshow('Image with Smoothed Shift Curve', img_display)
cv2.waitKey(0)
cv2.destroyAllWindows()