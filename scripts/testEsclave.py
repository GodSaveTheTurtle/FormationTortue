#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin, hypot, atan, degrees, radians, pi, copysign, sqrt, pow

# The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller.
# PID controller gives output value for error between desired reference input and measurement feedback to
# minimize error value.
# More information: http://en.wikipedia.org/wiki/PID_controller
#
# cnr437@gmail.com
#
# Example   #########
#
# p=PID(3.0,0.4,1.2)
# p.setPoint(5.0)
# while True:
#     pid = p.update(measurement_value)
#
#

maximumSpeed = 10


class PID:

    def __init__(self, P=2.0, I=0.0, D=1.0, PIDMax=5, Derivator=0,
                 Integrator=0, Integrator_max=500, Integrator_min=-500):
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
    D = dicoRobots['d']
    teta = radians(dicoRobots['theta'])
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
    dicoRobots['d'] = hypot(
        D * cos(teta) + signX * speed * cos(cap), D * sin(teta) + signY * speed * sin(cap))
    dicoRobots['theta'] = degrees(
        atan((D * sin(teta) + signX * speed * sin(cap)) / (D * cos(teta) + signY * speed * cos(cap))))


def main(dicoRobots, cap=0):
    # calcule geometriquement les cap et vitesses a atteindre
    # for i in range(len(dicoRobots)):
    currentSpeed = 0
    currentCap = 0
    teta1 = radians(dicoRobots.goal_theta_rad)
    teta2 = radians(dicoRobots.theta_rad)
    D1 = dicoRobots.goal_d
    D2 = dicoRobots.d
    # if teta2 > teta1:  # compare teta and tetaSetPoint for the pi/2 shift due to the fact that -pi/2<atan<pi/2
    #     angleShift = pi / 2
    # else:
    #     angleShift = -pi / 2
    ######
    # TODO: Erreur: division par 0
    ######
    # setCap = degrees(
    #     angleShift - atan((D2 * cos(teta2) - D1 * cos(teta1)) / (D2 * sin(teta2) - D1 * sin(teta1)))) / 180
    distanceToObjective = hypot((D1 * cos(teta1) - D2 * cos(teta2)), (
        D1 * sin(teta1) - D2 * sin(teta2))) / maximumSpeed

    """
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
    """
    time_out = distanceToObjective * 1.5  # temps laisse au robot pour atteindre l'objectif

    if time_out == 0:
        return 0, 0
    else:
        # si la vitesse est en quart de tours par minute, a verifier pour obtenir correlation
        angle = pi / 2 / time_out

        # but : faire parcourir un quart de cercle au robot
        speed = pi / 2 * sqrt(pow(distanceToObjective, 2) / 2) / time_out

        return (angle, speed)  # pour twist: x de linear speed (m/s), z de angular speed (rad/s)

if __name__ == '__main__':
    main()
