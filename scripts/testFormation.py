#!/usr/bin/env python
# -*- coding: ascii -*-
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

numberOfRobots = 2
setDistance = 3
# distance centurion-legionaire en metres
maximumDistance = setDistance * 1.1
# tolere 10pourcent d'erreur en trop et en moins
minimumDistance = setDistance * 0.9
sightWidth = 57
# angle du champ de vision en degres
tetaStep = sightWidth / numberOfRobots  # calcul du pas d'angle pour distribuer par la suite
tetaTolerance = tetaStep / 3  # domaine de tolerance sur angle  tout robot depassant considere comme a corriger
dicoRobots = {0: {'IP': '192.168.0.110', 'tetaSetPoint': 0, 'setDistance': 0, 'teta': 0, 'D': 0},
              1: {'IP': '192.168.0.110', 'tetaSetPoint': 0, 'setDistance': 0, 'teta': 0, 'D': 0}}
regulationActivated = True


def robotFlowValuesAquire(dicoRobots):
# Aquiert les donnees de position actuelles des robots de la kinnect
    for i in range(len(dicoRobots)):
        dicoRobots[i]['teta'] = 2 * pi / 6
        dicoRobots[i]['D'] = 2


def robotCommandValuesCompute(dicoRobots):
# calcule geometriquement les cap et vitesses a atteindre
    for i in range(len(dicoRobots)):
        teta1 = dicoRobots[i]['tetaSetPoint']
        teta2 = dicoRobots[i]['teta']
        D1 = dicoRobots[i]['setDistance']
        D2 = dicoRobots[i]['D']
        dicoRobots[i]['cap'] = pi / 2 - atan((D2 * cos(teta2) - D1 * cos(teta1)) / (D2 * sin(teta2) - D1 * sin(teta1)))
        dicoRobots[i]['speed'] = hypot((D1 * cos(teta1) - D2 * cos(teta2)), (D1 * sin(teta1) - D2 * sin(teta2)))


def modeRegulation(dicoRobots):
        robotFlowValuesAquire(dicoRobots)
        robotCommandValuesCompute(dicoRobots)


def robotPositionDomainSet(dicoRobots):
# calcule les domaines d'angles a atteindre en fonction du dico de robots
    tetaDotsDomain = []
    for i in range(int(len(dicoRobots)) + 1):
        tetaDotsDomain.append(i * tetaStep - 50)
    for i in range(len(dicoRobots)):
        dicoRobots[i]['tetaSetPoint'] = (tetaDotsDomain[i] + tetaDotsDomain[i + 1]) / 2
        dicoRobots[i]['setDistance'] = setDistance
        # ajout des domaines angles a respecter par chaque robot dans le dico
    #print('positionDomain:', dicoRobots)


"""def regulationMessagesFlow():# emet les consignes de cap et de vitesse a intervalles reguliers
    def __init__(self, target_sim=False):
            if target_sim:#
                    topic = '/turtle1/cmd_vel'
            else:
                    topic = '/cmd_vel_mux/input/teleop'
                super(DirectionController, self).__init__(topic, Twist, 1/10.0)
                self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.s.bind(('0.0.0.0', 1337))
    def loop(self):
        for i in range(numberOfRobots):
            vector=modeRegulation(i,idOfRobot)
            t=Twist()
            t.linear.x=vector[0]
            t.angular.z=vector[5]
            publier.publish(t)
            print tetaDotsDomain,robotsPositionsDomain,'termine'
        rospy.sleep(self.frequency)"""


def main():
    dicoRobots = {
        0: {'IP': '192.168.0.110', 'tetaSetPoint': 0, 'setDistance': 0, 'teta': 0, 'D': 0},
        1: {'IP': '192.168.0.110', 'tetaSetPoint': 0, 'setDistance': 0, 'teta': 0, 'D': 0}
    }
    print(dicoRobots)
    robotPositionDomainSet(dicoRobots)
    # while regulationActivated:
    modeRegulation(dicoRobots)
        # regulationMessagesFlow(dicoRobots)
    #print(dicoRobots)
if __name__ == '__main__':
    main()
