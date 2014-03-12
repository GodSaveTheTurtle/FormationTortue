#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sys
import tty
import termios
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import cv2.cv as cv
from math import cos, sin, hypot, atan, degrees, radians, pi
import testEsclave
import time


numberOfRobots = 1
goal_d = 3
# distance centurion-legionaire en metres
maximumDistance = goal_d * 1.1
# tolere 10pourcent d'erreur en trop et en moins
minimumDistance = goal_d * 0.9
sightWidth = 57
# angle du champ de vision en degres
tetaStep = sightWidth / numberOfRobots  # calcul du pas d'angle pour distribuer par la suite
tetaTolerance = tetaStep / 3  # domaine de tolerance sur angle  tout robot depassant considere comme a corriger
dicoRobots = {0: {'goal_theta': 0, 'goal_d': 0, 'theta': 0, 'd': 0}}
regulationActivated = True


def modeRegulation(dicoRobots):
        #robotFlowValuesAquire(dicoRobots)
        testEsclave.main(dicoRobots)


def robotPositionDomainSet(dicoRobots):
# calcule les domaines d'angles a atteindre en fonction du dico de robots
    tetaDotsDomain = []
    for i in range(int(len(dicoRobots)) + 1):
        tetaDotsDomain.append(i * tetaStep - sightWidth / 2)
    for i in range(len(dicoRobots)):
        dicoRobots[i]['goal_theta'] = (tetaDotsDomain[i] + tetaDotsDomain[i + 1]) / 2
        dicoRobots[i]['goal_d'] = goal_d
        # ajout des domaines angles a respecter par chaque robot dans le dico
    #print('positionDomain:', dicoRobots)


def test():
    dicoRobots = [
        {'goal_theta': 0, 'goal_d': 0, 'theta': 0, 'd': 0},
        {'goal_theta': 0, 'goal_d': 0, 'theta': 0, 'd': 0}
    ]

    run(dicoRobots)

    # Le dico est censé être envoyé sur le réseau et les fonctions suivantes exécutées sur chaque esclave
    while True:
        modeRegulation(dicoRobots[0])
        print(dicoRobots)
        time.sleep(5)
        print('position reset')
        # regulationMessagesFlow(dicoRobots)

    print(dicoRobots)


def run(dicoRobots):
    robotPositionDomainSet(dicoRobots)


if __name__ == '__main__':
    test()
