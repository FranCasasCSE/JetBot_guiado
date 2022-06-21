#Script para prueba de giro+traslación del robot

#Prueba triangular: 3 puntos

from robotClass_v8 import DifferentialRobot

move = DifferentialRobot(kp=[0.2,0.7], ki=[0.01, 0.001], dead_zone=[0])

def main():
    #Prueba triangular:robot colocado en x=0.4, y=0
    move.go_to(x=0.4, y=0.5)
    print('Punto 1 alcanzado')
    move.go_to(x=0.6, y=0.2)
    print('Punto 2 alcanzado')
    move.go_to(x=0.4, y=0)
    print('Punto 3 alcanzado - fin de ruta')
    move.stop()

main()

