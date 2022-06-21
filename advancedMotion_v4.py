#Script para prueba de giro

#Prueba de giro a 4 angulos

import math
import time
from robotClass_v8 import DifferentialRobot

move = DifferentialRobot(kp=[0.01,0.7], ki=[0.0005, 0.001], dead_zone=[0])
coefRadDeg = math.pi/180

def main():
    move.turn_to(45*coefRadDeg)
    print('Angulo 1 alcanzado')
    time.sleep(3)
    move.turn_to(180*coefRadDeg)
    print('Angulo 2 alcanzado')
    time.sleep(3)
    move.turn_to(315*coefRadDeg)
    print('Angulo 3 alcanzado')
    time.sleep(3)
    move.turn_to(135*coefRadDeg)
    print('Angulo 4 alcanzado')
    move.stop()


main()

