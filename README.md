FormationTortue
===============

## Scénario

- Les robots de la formation suivent une formation choisie (depuis un appareil android) et doivent suivre les instructions d'un leader (commandé manuellement depuis android) en conservant la formation.

- La formation est constituée d'un robot leader précédé par des robots esclaves disposés devant lui en arc de cercle.


## Structure, exécution

Le code est structuré en package ROS, comme recommandé dans la documentation:

```
formation/                # racine du package, respecte la convention de nommage de package python
|_ scripts/               # fichiers privés au package et exécutables
|_ src/                   # fichers publics, exportés par le package
|_ package.xml            # descripteur du package
|...                      # autres
```

** Newsflash: je n'arrive pas à faire marcher la com par topics en multi robots. Je vais donc utiliser des sockets et les manips ci-dessous ne sont pas indispensables. Exécuter les scripts python directement suffit, mais c'est moins pratique.**

### Lancement via `roslaunch`

Il faut d'abord configurer le workspace catkin et build le package:

```
~/../catkin_ws [1] $ ls src
formation

~/../catkin_ws [2] $ catkin_init_workspace
Creating symlink "/home/.../catkin_ws/CMakeLists.txt" pointing to "/opt/ros/hydro/share/catkin/cmake/toplevel.cmake"

~/../catkin_ws [3] $ catkin_make
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

~/../catkin_ws [5] $ cd src/formation

~/../catkin_ws [6] $ roslaunch maitre.launch

```

#### Commentaires

Les numéros ci-dessous font référence aux numéros de ligne du listing précédent

1. Le dépôt doit être dans des sources du workspace catkin.
2. Initialisation du workspace
3. Construction du package. Le but ici est principalement de récupérer le fichier setup, puisqu'on ne compile pas le python.
4. Charger le fichier setup. Il y en a différents disponibles, suivant le type de shell qu'on préfère. Il peut être pratique d'ajouter cette ligne (en corrigeant le chemin bien sûr) au .bashrc pour éviter de le faire à chaque création de shell.
5. Peut ça change peut être rien d'être ou pas dans le package...
6. Lancement du noeud maitre

### Lancement des scripts python directement

Les fichiers `.launch` permettent de définir des paramètres de démarrage et de lancer plusieurs noeuds en une seule commande. Pour les remplacer il faut donc faire tout cela manuellement.

* Initialiser correctement les variables en remplaçant les appels à `rospy.get_param()` par l'affectation désirée. (voir fonction `setup()` dans `master.py` et `slave.py`).

* Démarrer dans des terminaux séparés (ou en tâche de fond):
	- Le master node: `roslaunch turtlebot_bringup minimal.launch` ou `roscore`)
	- Le noeud kinect si on veut l'utiliser: `roslaunch openni_launch openni.launch`
	- Le noeud turtlesim si on veut l'utiliser: `rosrun turtlesim turtlesim_node`
	- Le maitre: `python master.py`
	- Des esclaves: `python slave.py`. Si la config des esclaves est ajoutée en dur dans le fichier, il en faut bien sûr plusieurs pour des esclaves différents.


## Réseau

* Les ports suivants sont utilisés:
	- UDP 1337: Communication entre le maître et l'appli android. Pour les simulations, on peut aussi déplacer le maître directement via turtlesim (lancer `rosrun turtlesim turtlesim_node`)
	- TCP 1338: Communication entre les esclaves et le maître. Pas nécessaire pour les simulations en local.