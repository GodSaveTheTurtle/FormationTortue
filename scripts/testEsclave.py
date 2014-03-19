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


def min_angle(angle_rad):
    if angle_rad < -pi:
        return 2 * pi + angle_rad
    elif angle_rad > pi:
        return angle_rad - 2 * pi
    else:
        return angle_rad


def main(dicoRobots, cap=0):
    # calcule geometriquement les cap et vitesses a atteindre
    # for i in range(len(dicoRobots)):
    angle = 0
    speed = 0
    master_cap = dicoRobots.master_theta_rad
    theta_obj = dicoRobots.goal_theta_rad - master_cap
    theta_esclave = dicoRobots.theta_rad - master_cap
    D_goal = dicoRobots.goal_d
    D_esclave = dicoRobots.d
    print ('capmaitr', degrees(master_cap), 'theta_obj', degrees(theta_obj), 'theta_esc', degrees(theta_esclave))

    # if theta_obj > theta_esclave:  # compare teta and tetaSetPoint for the pi/2 shift due to the fact that -pi/2<atan<pi/2
    #     angleShift = pi / 2
    # else:
    #     angleShift = -pi / 2
    #
    # TODO: Erreur: division par 0
    #
    # setCap = degrees(
    # angleShift - atan((D_goal * cos(theta_obj) - D_esclave *
    # cos(theta_esclave)) / (D_goal * sin(theta_obj) - D_esclave *
    # sin(theta_esclave)))) / 180

    distanceToObjective = hypot((D_goal * sin(theta_obj) - D_esclave * sin(theta_esclave)), (
        D_goal * cos(theta_obj) - D_esclave * cos(theta_esclave)))
    X_projection = D_goal * cos(theta_obj) - D_esclave * cos(theta_esclave)
    Y_projection = D_goal * sin(theta_obj) - D_esclave * sin(theta_esclave)
    if X_projection == 0 or Y_projection == 0:
        angle_to_objective = 0
    elif X_projection > 0:
        angle_to_objective = -atan(Y_projection / X_projection)
    elif X_projection < 0:
        if Y_projection > 0:
            angle_to_objective = - pi + atan(Y_projection / X_projection)
        else:
            angle_to_objective = pi + atan(Y_projection / X_projection)

        #angle_to_objective = (copysign(Y_projection, 1) * (pi - atan(Y_projection / -X_projection))) % (2 * pi)
    time_out = distanceToObjective * 3  # temps laisse au robot pour atteindre l'objectif

        # vitesse en rad/s
    if abs(cap - angle_to_objective) < 0.5:
        speed = distanceToObjective / 4
    else:
        if angle_to_objective > 0:
            angle = abs(cap - angle_to_objective) / 2
        elif angle_to_objective < 0:
            angle = -abs(cap - angle_to_objective) / 2
        speed = 0
         # angle objectif presque atteint : on peut avancer
    print('angle2obj:', degrees(angle_to_objective), 'capEsc', degrees(cap))
    print(dicoRobots)
    print('angle', degrees(angle), 'speed', speed)
    print('distance2obj', distanceToObjective)
    return (angle, speed)  # pour twist: x de linear speed (m/s), z de angular speed (rad/s)

"""
import math




mode_regulation = False
REGULATION_MIN_ANGLE = math.radians(15)
REGULATION_TRANSITION_ANGLE = math.radians(45)
TOLERANCE_D = 0.15
K_LIN = 0.5


def michaelangelo(dicoRobots, cap):
    global mode_regulation
    ang_spd, lin_spd = 0, 0

    delta_cap = min_angle(dicoRobots.master_theta_rad) - min_angle(cap)
    delta_theta = min_angle(dicoRobots.theta_rad) - min_angle(dicoRobots.goal_theta_rad)

    # print delta_cap, math.degrees(delta_cap), cap

    mode_regulation = ((mode_regulation or math.fabs(delta_cap) > REGULATION_TRANSITION_ANGLE) and
                       not math.fabs(delta_cap) < REGULATION_MIN_ANGLE)

    if mode_regulation:
        ang_spd = delta_cap
        lin_spd = 0
    else:
        ang_spd = delta_theta
        delta_d = dicoRobots.goal_d - dicoRobots.d

        if math.fabs(delta_d) < TOLERANCE_D:
            lin_spd = 0
        else:
            lin_spd = K_LIN * delta_d

    # print ang_spd, lin_spd, mode_regulation

    return ang_spd, lin_spd"""


def yay_trigo(dicoRobots, cap):
    theta_e = min_angle(dicoRobots.master_theta_rad - dicoRobots.theta_rad)
    theta_g = min_angle(dicoRobots.master_theta_rad - dicoRobots.goal_theta_rad)

    a = math.fabs(theta_g * math.cos(dicoRobots.goal_d) - theta_e * math.cos(dicoRobots.d))
    b = math.fabs(theta_g * math.sin(dicoRobots.goal_d) - theta_e * math.sin(dicoRobots.d))

    theta = math.pi/2 - math.atan2(a, b) - cap

    print a, b
    print math.degrees(theta), theta

    if math.fabs(theta) < math.radians(45):
        d = math.hypot(a, b)
    else:
        d = 0

    if math.fabs(d) < 1:
        d = 0
        theta = min_angle(dicoRobots.master_theta_rad) - min_angle(cap)

    return theta, d


if __name__ == '__main__':
    main()
