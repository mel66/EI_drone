import numpy as np
import cv2
from scipy.ndimage import median_filter


def draw_function(
    img,
    Y,                   # The values, len(Y) = img.shape[1]
    hmin, hmax,           # The curve is plotted between image lines hmin and hmax.
    ymin, ymax,          # The value range [ymin, ymax] is mapped into [hmin, hmax].
    color, thickness) : 
    # Tracer la courbe sur l'image
    if len(Y) != img.shape[1]:
        raise ValueError("La longueur de Y doit correspondre à la largeur de l'image.")
    # Calculer les coordonnées Y de la courbe
    scaled_Y = np.clip((Y - ymin) / (ymax - ymin), 0, 1)  # Normalisation entre 0 et 1
    curve_y = hmax - (scaled_Y * (hmax - hmin)).astype(int)  # Mapping entre hmin et hmax

    for x in range(1, len(Y)):
        pt1 = (x - 1, curve_y[x - 1])
        pt2 = (x, curve_y[x])
        cv2.line(img, pt1, pt2, color, thickness)
    return img
    


def local_shift(intensity_1, intensity_2, max_shift=10, window_radius=5):
    # Extraire la partie centrale d'intensity_1 pour éviter les débordements
    cut_intensity_1 = intensity_1[max_shift + window_radius : - (max_shift + window_radius)]
    
    # Définir les indices pour les positions x et les valeurs de décalage
    x_indices = np.arange(len(cut_intensity_1))
    s_values = np.arange(max_shift + 1)
    h_values = np.arange(-window_radius, window_radius + 1)

    # Ajuster les indices pour générer des fenêtres de positions
    pos = h_values + max_shift + window_radius
    pos1 = x_indices[:, None] + pos
    pos2 = x_indices[:, None, None] + s_values[None, :, None] + pos  # Création des fenêtres pour décalage

    # Extraire les fenêtres d'intensité pour chaque position
    try:
        intensity_1_window = intensity_1[pos1]  # (positions x, fenêtre)
        intensity_2_window = intensity_2[pos2]  # (décalages s, positions x, fenêtre)
    except IndexError as e:
        print(f"Erreur d'index : {e}")
        print(f"Dimensions de pos1 : {pos1.shape}, Dimensions de pos2 : {pos2.shape}")
        return None  # ou gérer l'erreur différemment

    # Calculer les différences au carré entre les deux fenêtres
    diff_squared = (intensity_1_window[:, None, :] - intensity_2_window) ** 2

    # Calculer la somme des différences au carré pour chaque (s, x)
    costs = diff_squared.sum(axis=-1)  # Somme sur la dimension fenêtre

    # Trouver le décalage `s` qui minimise le coût pour chaque position `x`
    optimal_shifts = costs.argmin(axis=1)

    return optimal_shifts


def detect_door(signal1, signal2, shift, padding, std_threshold=0.5, window_size=11, similarity_threshold=100):
    """
    Détecte la présence de mur (0) ou de porte/profondeur (1) en analysant les signaux de décalage.

    Parameters:
    - signal1 (np.ndarray): Première ligne d'intensité extraite de l'image précédente.
    - signal2 (np.ndarray): Deuxième ligne d'intensité extraite de l'image actuelle.
    - shift (np.ndarray): Tableau des décalages lissés entre signal1 et signal2.
    - window_size (int): Taille de la fenêtre glissante utilisée pour analyser les variations de `shift`.
    - similarity_threshold (float): Seuil pour déterminer si une section est un mur uniforme.

    Returns:
    - c_values (np.ndarray): Tableau indiquant 0 (mur) ou 1 (porte/profondeur) pour chaque point.
    """

    # Assurer que la taille de la fenêtre est impaire pour centrer la fenêtre
    assert window_size % 2 == 1, "window_size doit être impair pour centrer la fenêtre"
    half_window = window_size // 2

    # Initialiser le tableau des résultats
    c_values = np.zeros_like(shift)
    max_shift = max(shift)
    # Parcourir chaque point `c` avec une fenêtre glissante centrée
    for c in range(padding, len(shift) - padding):
        if shift[c] >= max_shift//2:
            c_values[c] = 0
            pass
        else:
            # Définir la fenêtre centrée en `c`
            window1 = signal1[c - half_window : c + half_window + 1]
            window2 = signal2[c - half_window : c + half_window + 1]

            # Calculer l'écart-type pour chaque fenêtre
            std_signal1 = np.std(window1)
            std_signal2 = np.std(window2)
            
            # Vérifier si les écarts-types sont faibles (indiquant un mur uniforme)
            if std_signal1 < std_threshold and std_signal2 < std_threshold:
                c_values[c] = 0  # Mur détecté
            else:
                c_values[c] = 1  # Porte ou profondeur détectée

    c_values = median_filter(c_values, size=40)  # Ajustez la taille de la fenêtre selon vos besoins

    return c_values