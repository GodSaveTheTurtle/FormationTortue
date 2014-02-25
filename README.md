FormationTortue
===============

## Scénario

- Les robots de la formation suivent une formation choisie (depuis un appareil android) et doivent suivre les instructions d'un leader (commandé manuellement depuis android) en conservant la formation.

- La formation est constituée d'un robot leader précédé par des robots esclaves disposés devant lui en arc de cercle.


## Commandes utiles ROS

* `rqt_console` Affiche les loginfo sur rosout
* `roslaunch turtlebot_bringup minimal.launch` Démarre le master node
* `rosrun image_view disparity_view image:=/camera/depth/disparity` Affiche la vue colorisée kinect
* `rosrun openni_launch openni.launch` Démarre le noeud kinect
* `rosrun turtlesim turtlesim_node`
* `rosrun turtlesim turtle_teleop_key`

