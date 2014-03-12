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
import time
from math import cos, sin, hypot, atan, degrees, radians, pi, copysign


# The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
# More information: http://en.wikipedia.org/wiki/PID_controller
#
# cnr437@gmail.com
#
# Example	#########
#
# p=PID(3.0,0.4,1.2)
# p.setPoint(5.0)
# while True:
#     pid = p.update(measurement_value)
#
#

i = 0
maximumSpeed = 10


class PID:

    def __init__(self, P=2.0, I=0.0, D=1.0, PIDMax=5, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.PIDMax = PIDMax
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        if PID > self.PIDMax:
            PID = self.PIDMax
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        print('cap was set to', set_point)
        self.Integrator = 0
        self.Derivator = 0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator


class firstOrder:

    def __init__(self, G=0.5, T=100, maxF=5):
        self.G = G
        self.T = T
        self.set_point = 0.0
        self.error = 0.0
        self.maxFlow = maxF
    def update(self, current_value):
        flowMeasured = current_value + \
            (self.set_point - current_value) * self.G + (self.set_point - current_value) / self.T
        if abs(flowMeasured) > self.maxFlow:
            flowMeasured = copysign(1.0, flowMeasured) * self.maxFlow
        return flowMeasured

    def setPoint(self, set_point):
        self.set_point = set_point
        print('fvalue was set to', set_point)

    def setKp(self, G):
        self.G = G

    def getPoint(self):
        return self.set_point


def updatePosition(currentSpeed, currentCap, dicoRobots):
    D = dicoRobots[i]['D']
    teta = radians(dicoRobots[i]['teta'])
    speed = currentSpeed
    cap = currentCap * pi
    if cap > 0 and cap < pi / 2:
        signX = -1
        signY = 1
    elif cap > 0 and cap > pi / 2:
        signX = -1
        signY = 1
    elif cap < 0 and cap < pi / 2:
        signX = -1
        signY = 1
    elif cap < 0 and cap > pi / 2:
        signX = -1
        signY = 1
    else:
        signX = 1
        signY = 1
    dicoRobots[i]['D'] = hypot(
        D * cos(teta) + signX * speed * cos(cap), D * sin(teta) + signY * speed * sin(cap))
    dicoRobots[i]['teta'] = degrees(
        atan((D * sin(teta) + signX * speed * sin(cap)) / (D * cos(teta) + signY * speed * cos(cap))))


def main(dicoRobots):
    # calcule geometriquement les cap et vitesses a atteindre
    # for i in range(len(dicoRobots)):
    currentSpeed = 0
    currentCap = 0
    teta1 = radians(dicoRobots[i]['tetaSetPoint'])
    teta2 = radians(dicoRobots[i]['teta'])
    D1 = dicoRobots[i]['setDistance']
    D2 = dicoRobots[i]['D']
    if teta2 > teta1:  # compare teta and tetaSetPoint for the pi/2 shift due to the fact that -pi/2<atan<pi/2
        angleShift = pi / 2
    else:
        angleShift = -pi / 2
    setCap = degrees(
        angleShift - atan((D2 * cos(teta2) - D1 * cos(teta1)) / (D2 * sin(teta2) - D1 * sin(teta1)))) / 180
    setSpeed = hypot((D1 * cos(teta1) - D2 * cos(teta2)), (D1 * sin(teta1) - D2 * sin(teta2))) / maximumSpeed
    print('setCap', i, setCap)
    print('speed', i, setSpeed)
    pidForSpeed = PID(0.5, 0.4, 0.5, 10)   # P, I, D
    pidForSpeed.setPoint(setSpeed)  # valeur a suivre
    processForSpeed = firstOrder(0.5, 5)

    pidForCap = PID(0.5, 0.4, 0.5, 1)   # P, I, D
    pidForCap.setPoint(setCap)   # valeur a suivre
    processForCap = firstOrder(0.5, 1)
    N = 10
    while N > 0:
        flowSpeed = pidForSpeed.update(currentSpeed)
        processForSpeed.setPoint(flowSpeed)
        currentSpeed = processForSpeed.update(currentSpeed)
        flowCap = pidForCap.update(currentCap)
        processForCap.setPoint(flowCap)
        currentCap = processForCap.update(currentCap)
        print('sCap', setCap, 'cCap', currentCap)
        print('sSpeed', setSpeed, 'cSpeed', currentSpeed)
        updatePosition(currentSpeed, currentCap, dicoRobots)
        #time.sleep(0.1)
        N = N - 1
        print(N)

if __name__ == '__main__':
    main()
