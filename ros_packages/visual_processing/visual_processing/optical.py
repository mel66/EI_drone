import numpy as np
import cv2
from scipy.ndimage import median_filter, binary_closing, label


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
    


def local_shift(i1, i2, max_shift, window_radius):
    # Extraire la partie centrale d'intensity_1 pour éviter les débordements
    cut_intensity_1 = i1[max_shift + window_radius : - (max_shift + window_radius)]
    
    # Définir les indices pour les positions x et les valeurs de décalage
    x_indices = np.arange(len(cut_intensity_1))
    s_values = np.arange(max_shift + 1)
    h_values = np.arange(-window_radius, window_radius + 1)

    # Ajuster les indices pour générer des fenêtres de positions
    pos = h_values + max_shift + window_radius
    pos1 = x_indices[:, None] + pos
    pos2 = x_indices[:, None, None] + s_values[None, :, None] + pos  # Création des fenêtres pour décalage
    # Extraire les fenêtres d'intensité pour chaque position
    intensity_1_window = i1[pos1]  # (positions x, fenêtre)
    intensity_2_window = i2[pos2]  # (x, s, fenêtre)
    
    
    # Calculer les différences au carré entre les deux fenêtres
    diff_squared = (intensity_1_window[:, None, :] - intensity_2_window) ** 2

    # Calculer la somme des différences au carré pour chaque (s, x)
    costs = diff_squared.sum(axis=-1)  # Somme sur la dimension fenêtre

    # Trouver le décalage `s` qui minimise le coût pour chaque position `x`
    optimal_shifts = costs.argmin(axis=1)

    padding = max_shift + window_radius
    optimal_shifts = np.pad(optimal_shifts, (padding, padding), mode='constant', constant_values=0)
    return optimal_shifts


def detect_door( signal2, shift, padding, window_size=5):
    """
    Détecte la présence de mur (0) ou de porte/profondeur (1) en analysant les signaux de décalage.
    Conserve uniquement le plus gros palier détecté pour chaque type (0 ou 1) sans boucle for.

    Parameters:
    - signal1 (np.ndarray): Première ligne d'intensité extraite de l'image précédente.
    - signal2 (np.ndarray): Deuxième ligne d'intensité extraite de l'image actuelle.
    - shift (np.ndarray): Tableau des décalages lissés entre signal1 et signal2.
    - window_size (int): Taille de la fenêtre glissante utilisée pour analyser les variations de `shift`.
    - threshold (float): Seuil pour déterminer si une section est un mur uniforme.

    Returns:
    - c_values (np.ndarray): Tableau indiquant 0 (mur) ou 1 (porte/profondeur) pour chaque point.
    """

    ratio_bas = 0.35
    percentile_percent = 4*window_size
    width_threshold = (len(signal2) * 0.30)
    med_filter = 30


    assert window_size % 2 == 1, "window_size doit être impair pour centrer la fenêtre"
    half_window = window_size // 2

    # Initialiser le tableau des résultats
    c_values = np.zeros_like(shift)
    max_shift = max(shift)

    #Pour chacun de ces points avec faible shift, construire une fenetre autour
    #Prendre np.std de ces fenetres
    #Mettre à 1 si std > threshold

    # Définir les indices des valeurs centrales (en excluant les valeurs de bord)
    central_indices = np.arange(padding, len(shift) - padding)
    # Création d'un masque pour les zones centrales où le shift est bas
    low_shift_mask = (shift[central_indices] <= max_shift * ratio_bas)
    low_shift_indices = central_indices[low_shift_mask]

    # Création de toutes les fenêtres pour les indices centraux de faible shift
    windows = np.lib.stride_tricks.sliding_window_view(signal2, window_size)[low_shift_indices - half_window]

    # Calculer l'écart-type sur chaque fenêtre de manière vectorisée
    std_values = np.std(windows, axis=1)
    if len(std_values) > 0:
        similarity_threshold = np.percentile(std_values,percentile_percent)
        # Mettre à 1 les indices centraux où l'écart-type est supérieur au seuil
        c_values[low_shift_indices] = (std_values > similarity_threshold).astype(int)
        c_values = median_filter(c_values, size=med_filter)
    else:
        c_values[:] =  0
     

    # Appliquer une dilatation binaire pour fusionner les segments proches
    #structure = np.ones(60)
    #c_values = binary_closing(c_values, structure=structure).astype(int)
    # Identifier les segments de `1` et calculer leur largeur
    labeled_array, num_features = label(c_values == 1)
    for i in range(1, num_features + 1):
        segment_width = np.sum(labeled_array == i)
        # Garder uniquement les pics avec une largeur supérieure au seuil
        if segment_width <= width_threshold:
            c_values[labeled_array == i] = 0    
    return (c_values)
