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
    theta2 = min(dicoRobots.goal_theta_rad, 2 * pi - dicoRobots.goal_theta_rad)
    theta1 = min(dicoRobots.theta_rad, 2 * pi - dicoRobots.theta_rad)
    D2 = dicoRobots.goal_d
    D1 = dicoRobots.d
    master_cap = dicoRobots.master_theta_rad
    # if theta2 > theta1:  # compare teta and tetaSetPoint for the pi/2 shift due to the fact that -pi/2<atan<pi/2
    #     angleShift = pi / 2
    # else:
    #     angleShift = -pi / 2
    #
    # TODO: Erreur: division par 0
    #
    # setCap = degrees(
    #     angleShift - atan((D2 * cos(theta2) - D1 * cos(theta1)) / (D2 * sin(theta2) - D1 * sin(theta1)))) / 180

    distanceToObjective = hypot((D2 * sin(theta2) - D1 * sin(theta1)), (
        D2 * cos(theta2) - D1 * cos(theta1)))
    X_projection = D2 * cos(theta2) - D1 * cos(theta1)
    Y_projection = D2 * sin(theta2) - D1 * sin(theta1)
    if X_projection == 0 or Y_projection == 0:
        angle_to_objective = 0
    elif X_projection > 0:
        if Y_projection > 0:
            angle_to_objective = master_cap + atan(Y_projection / X_projection) % (2 * pi)
        else:
            angle_to_objective = master_cap + (-pi / 2 + atan(abs(Y_projection / X_projection))) % (2 * pi)
    elif X_projection < 0:
        if Y_projection > 0:
            angle_to_objective = master_cap + (pi - atan(abs(Y_projection / X_projection))) % (2 * pi)
        else:
            angle_to_objective = master_cap + (-pi + atan(abs(Y_projection / X_projection))) % (2 * pi)
        #angle_to_objective = (copysign(Y_projection, 1) * (pi - atan(Y_projection / -X_projection))) % (2 * pi)
    time_out = distanceToObjective * 3  # temps laisse au robot pour atteindre l'objectif

    if time_out < 0.5:
        angle = 0
        speed = 0
        print('angle:', degrees(angle_to_objective), 'cap', degrees(cap))
        return (angle, speed)
    else:
        # vitesse en rad/s
        if cap > angle_to_objective * 1.1:
            angle = -abs(cap - angle_to_objective)
        elif cap < angle_to_objective * 0.9:
            angle = abs(cap - angle_to_objective)
        else:
            angle = 0
        # but : faire parcourir un quart de cercle au robot
        if angle == 0 and distanceToObjective > 0.25:
            speed = distanceToObjective / 4  # angle_to_objective * sqrt(pow(distanceToObjective, 2) / 2) / time_out
        else:
            speed = 0
        print('angle2obj:', degrees(angle_to_objective), 'cap', degrees(cap))
        print('teta_robot', degrees(dicoRobots.theta_rad))
        print(dicoRobots)
        print('angle', degrees(angle), 'speed', speed)
        print('distance2obj', distanceToObjective)
        return (angle, speed)  # pour twist: x de linear speed (m/s), z de angular speed (rad/s)

if __name__ == '__main__':
    main()
