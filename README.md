FormationTortue
===============

## Scénario

- Les robots de la formation suivent une formation choisie (depuis un appareil android) et doivent suivre les instructions d'un leader (commandé manuellement depuis android) en conservant la formation.

- La formation est constituée d'un robot leader précédé par des robots esclaves disposés devant lui en arc de cercle.


## Commandes utiles ROS

* `rqt_console` Affiche les loginfo sur rosout
* `roslaunch turtlebot_bringup minimal.launch` Démarre le master node
* `rosrun image_view disparity_view image:=/camera/depth/disparity` Affiche la vue colorisée kinect
* `roslaunch openni_launch openni.launch` Démarre le noeud kinect
* `rosrun turtlesim turtlesim_node`
* `rosrun turtlesim turtle_teleop_key`

## Structure, exécution

Le code est structuré en package ROS, comme recommandé dans la documentation:

```
formation/                # racine du package, respecte la convention de nommage de package python
|_ scripts/               # fichiers privés au package et exécutables
|_ src/                   # fichers publics, exportés par le package
|_ package.xml            # descripteur du package
|...                      # autres
```

Il est possible d'exécuter le code simplement en exécutant les fichiers python, mais il est aussi possible d'utiliser directement les fichiers `.launch`. Pour cela il faut d'abord configurer le workspace catkin et build le package:

```
~/../catkin_ws [1] $ ls
formation

~/../catkin_ws [2] $ catkin_init_workspace
Creating symlink "/home/.../catkin_ws/CMakeLists.txt" pointing to "/opt/ros/hydro/share/catkin/cmake/toplevel.cmake"

~/../catkin_ws [3] $ catkin_make --source=.
...
-- +++ processing catkin package: 'formation'
-- ==> add_subdirectory(formation)
-- Configuring done
-- Generating done
-- Build files have been written to: /home/../catkin_ws/build
####
#### Running command: "make -j4 -l4" in "/home/../catkin_ws/build"
####

~/../catkin_ws [4] $ source devel/setup.bash

~/../catkin_ws [5] $ cd formation

~/../catkin_ws [6] $ roslaunch maitre.launch

```

#### Commentaires

1. Le dépôt doit être un sous dossier du workspace catkin. Si possible le renommer `formation` peut être une bonne idée (c'est le nom du package de toute façon)
2. Initialisation du workspace
3. Construction du package. Le but ici est principalement de récupérer le fichier setup, puisqu'on ne compile pas le python.
4. Charger le fichier setup. Il y en a différents disponibles, suivant le type de shell qu'on préfère. Il peut être pratique d'ajouter cette ligne (en corrigeant le chemin bien sûr) au .bashrc pour éviter de le faire à chaque création de shell.
6. Lancement du noeud maitre