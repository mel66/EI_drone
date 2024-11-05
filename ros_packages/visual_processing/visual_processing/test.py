import cv2
import numpy as np
import my_line_library as mll  # Import de la bibliothèque contenant la fonction shift

# Charger les deux frames
frame1 = cv2.imread("frame_t1.jpg")  # Remplacez par le chemin de votre première image
frame2 = cv2.imread("frame_t2.jpg")  # Remplacez par le chemin de votre deuxième image

# Vérifiez que les images ont bien été chargées
if frame1 is None or frame2 is None:
    raise FileNotFoundError("Assurez-vous que les chemins des images sont corrects et que les fichiers existent.")

# Convertir les frames en niveaux de gris
gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

# Extraire la ligne médiane de chaque frame
middle_row1 = gray1.shape[0] // 2
middle_row2 = gray2.shape[0] // 2
line1 = gray1[middle_row1, :]
line2 = gray2[middle_row2, :]

# Définir les paramètres pour la fonction shift
max_shift = 10  # Maximum shift à tester (ajustez selon vos besoins)
window_radius = 5  # Rayon de la fenêtre locale (ajustez selon vos besoins)

# Calculer le décalage entre les deux lignes
shift = mll.local_shift(line1, line2, max_shift=max_shift, window_radius=window_radius)

# Afficher le résultat
print(f"Calculated shift: {shift}")