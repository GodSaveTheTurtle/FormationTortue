#!/usr/bin/env python
# -*- coding: utf-8 -*-

import testEsclave
import time

from data_utils import SlaveData

# Les variables censées être utilisées uniquement à l'intérieur du module (donc privées) doivent commencer par '_'
_numberOfRobots = 1
_goal_d = 3
# distance centurion-legionaire en metres
_maximumDistance = _goal_d * 1.1
# tolere 10pourcent d'erreur en trop et en moins
_minimumDistance = _goal_d * 0.9
_sightWidth = 57
# angle du champ de vision en degres
_tetaStep = _sightWidth / _numberOfRobots  # calcul du pas d'angle pour distribuer par la suite
_tetaTolerance = _tetaStep / 3  # domaine de tolerance sur angle  tout robot depassant considere comme a corriger


def modeRegulation(dicoRobots):
        #robotFlowValuesAquire(dicoRobots)
        testEsclave.main(dicoRobots)


def robotPositionDomainSet(dicoRobots):
# calcule les domaines d'angles a atteindre en fonction du dico de robots
    tetaDotsDomain = []
    for i in range(len(dicoRobots) + 1):
        tetaDotsDomain.append(i * _tetaStep - _sightWidth / 2)
    i = 0
    for key in dicoRobots:
        dicoRobots[key].goal_theta = (tetaDotsDomain[i] + tetaDotsDomain[i + 1]) / 2
        dicoRobots[key].goal_d = _goal_d
        i += 1
        # ajout des domaines angles a respecter par chaque robot dans le dico
    #print('positionDomain:', dicoRobots)


def test():
    dicoRobots = {
        'yellow': SlaveData(),
        'pink': SlaveData()
    }

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
