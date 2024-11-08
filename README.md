# Projet EI Drone

Voici un lien vers la page des drones : [Page des drones](https://frezza.pages.centralesupelec.fr/st5-drones/EI/index.html)


Ce projet contient le code pour le contrôle du drone Parrot à l’aide de ROS 2. Il est conçu pour fonctionner avec un drone Parrot équipé de son driver ROS et une manette pour le contrôle manuel.

## Matériel requis

- **Drone Parrot** (avec driver ROS installé)
- **Manette de contrôle** (compatible avec le drone Parrot)

## Instructions d'installation et d'utilisation

### 1. Placer le terminal dans le répertoire du projet

cd EI_drone

### 2. Compiler le projet

colcon build

### 3. Sourcer l'environnement ROS 2

Après la compilation, sourcez l'environnement pour que les nouveaux packages soient reconnus :
source install/setup.bash

### 4. Lancer le projet

Utilisez le fichier de lancement ROS 2 pour initialiser les comportements du drone :

ros2 launch behavior behavior_launch.xml




