#!/usr/bin/env python
# -*- coding: utf-8 -*-

from math import cos, sin, hypot, atan, degrees, radians, pi, copysign, sqrt, pow
import math

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

maximumSpeed = 2  # définition en global de la vitesse linéaire maximale

# le PID devait aussi servir pour affiner le contrôle de trajectoire et de la vitesse :
# les consignes sont obtenues du main() sous forme de cap et de vitesse,
# et les commandes sont obtenues grâce au correcteur sous forme de vitesses angulaires et linéaires
# Elles devaient êtres envoyées sous forme de messages Twist au turtlebot


class PID:

    def __init__(self, P=2.0, I=0.0, D=1.0, PIDMax=5, Derivator=0,
                 Integrator=0, Integrator_max=500, Integrator_min=-500):  # définition des paramètres du PID
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

    def update(self, current_value):  # réactualisation de la consigne en fonction de l'erreur et des paramètres du pid
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

    def setPoint(self, set_point):   # définition de la consigne de vitesse ou de cap à atteindre
        self.set_point = set_point
        print('cap/speed was set to', set_point)
        self.Integrator = 0
        self.Derivator = 0

    def setIntegrator(self, Integrator):  # définition de la grandeur d'intégration
        self.Integrator = Integrator

    def setDerivator(self, Derivator):  # définition de la grandeur de dérivation
        self.Derivator = Derivator

    def setKp(self, P):  # définition du coefficient proportionnel
        self.Kp = P

    def setKi(self, I):  # définition du coefficient d'intégration
        self.Ki = I

    def setKd(self, D):  # définition du coefficient de dérivation
        self.Kd = D


class firstOrder:   # en vue de simuler le processus régulé par le correcteur sans utiliser turtle_sim

    def __init__(self, G=0.5, T=100, maxF=5):
        self.G = G  # gain statique
        self.T = T  # constante de temps
        self.set_point = 0.0  # initialisation à 0 de la variable de sortie
        self.error = 0.0  # mise à 0 de la variable d'entree ou de l'erreur
        self.maxFlow = maxF  # valeur de saturation de la grandeur de sortie (angle ou vitesse)

    def update(self, current_value):
    # mise à jour de la valeur en sortie du procédé en fonction des valeurs en entrée et des paramètres
        flowMeasured = current_value + \
            (self.set_point - current_value) * self.G + (self.set_point - current_value) / self.T
        if abs(flowMeasured) > self.maxFlow:
            flowMeasured = copysign(1.0, flowMeasured) * self.maxFlow
        return flowMeasured

    def setPoint(self, set_point):
    # définit une consignau procédé : on lui envoie la sortie du correcteur pour faire un système bouclé
        self.set_point = set_point
        print('fvalue was set to', set_point)

    def setKp(self, G):
        self.G = G

    def getPoint(self):
        return self.set_point


def min_angle(angle_rad):    # permet de travailler avec des angles normalisé [-pi;pi]
    if angle_rad < -pi:
        return 2 * pi + angle_rad
    elif angle_rad > pi:
        return angle_rad - 2 * pi
    else:
        return angle_rad


def deterministe_bis(dicoRobots, cap=0):
    # calcule geometriquement les cap et vitesses a atteindre
    # for i in range(len(dicoRobots)):
    angle = 0
    speed = 0
    master_cap = min_angle(dicoRobots.master_theta_rad)
    theta_obj = min_angle(min_angle(dicoRobots.goal_theta_rad) + master_cap)
    theta_esclave = min_angle(min_angle(dicoRobots.theta_rad) + master_cap)
    D_goal = dicoRobots.goal_d
    D_esclave = dicoRobots.d
    cap = min_angle(cap)
    if (theta_obj - theta_esclave) > 0:  # cap esclave trop élevé (=angle de dérive positif)
        if D_esclave > D_goal:
            angle_to_objective = master_cap - 3 * pi / 4  # le cap est déterminé selon quatre cas
        else:
            angle_to_objective = master_cap - pi / 4
    else:
        if D_esclave > D_goal:
            angle_to_objective = master_cap + 3 * pi / 4
        else:
            angle_to_objective = master_cap + pi / 4
    print('capmaitr', degrees(master_cap), 'theta_obj', degrees(theta_obj), 'theta_esc', degrees(theta_esclave))
    X_projection = D_goal * cos(theta_obj) - D_esclave * cos(theta_esclave)
    Y_projection = D_goal * sin(theta_obj) - D_esclave * sin(theta_esclave)
    distanceToObjective = hypot(X_projection, Y_projection)
    # vitesse en rad/s
    # cap: cap actuel, angle_to_ohjective = cap à atteindre
    if abs(cap - min_angle(angle_to_objective)) < 0.5:  # condition pour avoir une vitesse non nulle
        speed = distanceToObjective / 2  # vitesse non nulle car le cap à obtenir est suffisament proche du cap actuel
    else:  # cap à obtenir tro éligné : on pivote sans avancer
        if angle_to_objective > 0 and distanceToObjective > 0.5:
            angle = abs(cap - min_angle(angle_to_objective)) / 2  # moitié du delta entre cap actuel et
        elif angle_to_objective < 0 and distanceToObjective > 0.5:
            angle = -abs(cap - min_angle(angle_to_objective)) / 2
        speed = 0
         # angle objectif presque atteint : on peut avancer
    print('angle2obj:', degrees(angle_to_objective), 'capEsc', degrees(cap))
    print(dicoRobots)
    print('angle', degrees(angle), 'speed', speed)
    print('distance2obj', distanceToObjective)
    return (angle, speed)  # pour twist: x de linear speed (m/s), z de angular speed (rad/s)


def trigonometrique(dicoRobots, cap=0):
    angle = 0
    speed = 0
    master_cap = dicoRobots.master_theta_rad
    theta_obj = dicoRobots.goal_theta_rad - master_cap
    theta_esclave = dicoRobots.theta_rad - master_cap
    D_goal = dicoRobots.goal_d
    D_esclave = dicoRobots.d
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


mode_regulation = False
REGULATION_MIN_ANGLE = math.radians(15)
REGULATION_TRANSITION_ANGLE = math.radians(45)
TOLERANCE_D = 0.15
K_LIN = 0.5


def deterministe(dicoRobots, cap):
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

    return ang_spd, lin_spd  # renvoi des consignes de vitesse


def trigonometrique_bis(dicoRobots, cap):  # deuxième tentative de résolution par calcul trigonométrique
    theta_e = min_angle(dicoRobots.master_theta_rad - dicoRobots.theta_rad)  # angle vers esclave
    theta_g = min_angle(dicoRobots.master_theta_rad - dicoRobots.goal_theta_rad)  # angle vers objectif

    a = math.fabs(dicoRobots.goal_d * math.cos(theta_g) - dicoRobots.d * math.cos(theta_e))
    b = math.fabs(dicoRobots.goal_d * math.sin(theta_g) - dicoRobots.d * math.sin(theta_e))

    theta = math.pi / 2 - math.atan2(a, b) - cap

    print a, b
    print math.degrees(theta), theta

    if math.fabs(theta) < math.radians(45):
        d = math.hypot(a, b)
    else:
        d = 0

    if math.fabs(d) < 1:
        d = 0
        theta = min_angle(dicoRobots.master_theta_rad) - min_angle(cap)

    return theta, d # renvoi du theta pour application en consignes de vitesse


if __name__ == '__main__':
    main()
