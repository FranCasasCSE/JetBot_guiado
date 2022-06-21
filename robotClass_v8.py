#############################################################
#                                                           #
# Class for movement algorithm of Marvel IndoorGPS system   #
# combined with Jetbot                                      #
#                                                           #
# Author: Fran Casas                                        #
#                                                           #
# Version: 4                                                #
# Fecha: 18-06-2022                                         #
#                                                           #
#                                                           #
#############################################################

##CHANGELOG##
#V0 (13-06-2022):
# Definidos atributos de la clase DifferentialRobot:
#  DifferentialRobot(kp=0.5, ki=0.5, dead_zone=[0]):
# .testWork(val=1) -- si se usa este atributo la clase indicará que se han cargado correctamente los módulos
# .set_dead_zone -- se encuentra en desarrollo
# .obtener_posicion_angulo -- carga en las variables globales RAWimu e IMUfusion los valores devueltos por la baliza
# .obtener_yaw -- calcula el angulo z según los cuateriones (way=True) o según el giroscopio + acelerómetro (way=False)
# .turn_to -- incorpora un PI de corrección de ángulo, para el primer giro (falta ver efecto de la deriva)
# .go_to -- incorpora un doble PI para alcanzar la ubicación corrigiendo angulo + distancia al objetivo
# .take_sample -- guarda una imagen en la carpeta que se indique

#V1:
# -Corregido ángulo objetivo (initial_rot) sabiendo que queremos que el eje Y es el que gire (es donde apuntará la parte
# -delantera del robot con la cámara)
# -Cambio de signos en los PIs para obedecer el sentido de giro de las ruedas al corregir el movimiento (stop())

#V2:
# -Se corrige el paso de parámetros a las definiciones internas de la clase

#V3:
# -Se cambia la topología de las balizas a paired hedge (mediante cambio en dashboard)
# -Se modifica la clase MarvelmindHedge para incluir un método que devuelve exclusivamente ángulo (.return_angle)
# -Se adapta el código al nuevo método de obtención de orientación

#V4:
# -Arreglos menores: limites no aplicados en saturacion de PID pos + ang
# -Cambio de aplicación de señal en lugar de PWM continua a tren de pulsos
# -Incluida posibilidad de hacer logging de datos editando la variable logSet
# -Optimización de tiempo de giro según cuadrante
# -Se incorpora una variable SS que permite al controlador de posicion + angulo 
# funcionar en modo corta distancia (si SS=True) o en espacio amplio (SS=False)
# -Se incluyen puntos de control para ver salida de controladores
# -Se corrige la clase MarvelmindHedge (marvelmind_mod.py) para no devolver angulos negativos
# al devolver el angulo del eje Y robot (direccion de avance)
# -Dentro de dashboard se incrementa la velocidad de la comunciación radio a 154kbps para 
# incrementar la frecuencia de actualizacion de medidas
# -Se sustituye la baliza N1 por otra, debido a su malfuncionamiento
# -Corrección del doble PI de posicion+angulo, que no actualizaba la nueva direccion 
# a la que reorientar el robot
# -Se limita la señal PWM en turn_to a 0.3 para evitar altas velocidades (editable
# en la variable PWMmaxP), y a 0.35 en go_to, para evitar lo mismo pero permitir una velocidad
# algo mayor (no excesiva)

#V5:
# -Se incluye el guardado de posicion instantánea en logging de ruta (angulo+posicion)
# -Se elimina un retardo en el método go_to() que provocaba mayor lentitud en la ejecucion de ruta

#V6:
# -Se corrigen los fallos de guardado puntuales del csv debidos a no especificar ruta
# -Se incluye un método para detener tanto el puerto serie como la ejecución en curso 

#V7:
# -Se cambia la estructura de los controladores para poder usarlos con PWM continuo y disminuir la latencia
# -Se usan controladores separados para ángulo y posición (antes compartían parámetros)
# -Se usa una variable debug para devolver resultados por pantalla si se desea
# -Se optimiza levemente el código para eliminar operaciones repetidas

#V8:
# -Se elimina el uso de variables globales, al usar variables internas de la clase
# -Se crea una clase externa que alberga los PI

# IPython Libraries for display and widgets
import re
import ipywidgets
import traitlets
import ipywidgets.widgets as widgets
from IPython.display import display

#from turtle import up
# Camera and Motor Interface for JetBot
from jetbot import Robot, Camera, bgr8_to_jpeg
from marvelmind_mod import MarvelmindHedge
import math
import sys
import array
import numpy as np
from uuid import uuid1
import os
import json
import glob
import datetime
import cv2
import time
import csv
from classPI import classPI

#En primer lugar se debe hacer la lectura inicial tanto de coordenadas como de 
#ángulo, para calcular: la diferencia de coordenadas y el incremento de ángulo

class DifferentialRobot():
    def __init__ (self, kp,ki,dead_zone):
        self.ki = ki #cte integral del PI [kiAng, kiPos]
        self.kp = kp #cte proporcional del PI [kpAng, kpPos]
        self.dead_zone = dead_zone
        self.hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=6, debug=False) # create MarvellogSetmindHedge thread
        if (len(sys.argv)>1):
            self.hedge.tty= sys.argv[1]
        self.hedge.start() # start thread

        #Control de motores
        self.move = Robot()

        #Logging setup
        self.logSet=0 #Si logSet=0 no se hace logging de datos
                #Si logSet=1 se guardan los datos de posicionamiento a ángulo
                #Si logSet=2 se guardan los datos de angulo+posicion
        self.data=[] #Guardará los datos que se quieran colocar al CSV
        self.debug=True #Permite que se hagan todos los print de los metodos
        self.debugTime=True #Debug de tiempos de ejecucion

        #Memoria de KFs
        self.pos_x_prev=0
        self.pos_y_prev=0
        self.ang_int_prev=0
        self.timePrev=0
        self.accAngZprev=0
        self.y_pred1 = 0
        self.m_err1 = 0
        self.a1 = 0
        self.sigma_u1=2*10**(-7) #Varianza de la señal de posicion
        self.sigma_n1=0.01 #Varianza del ruido aplicado a la señal de entrada
        self.y_pred2 = 0
        self.m_err2 = 0
        self.a2 = 0
        self.sigma_u2=41.699 #Varianza de la señal de brujula
        self.sigma_n2=0.01
        self.y_pred3 = 0
        self.m_err3 = 0
        self.a3 = 0
        self.sigma_u3=83.602 #Varianza de la señal de gyro
        self.sigma_n3=0.01
        
        #Control de calculo de yaw (si se usa)
        self.way=True #False=metodo 1, correcion de deriva
                #True=metodo 2, estimacion por cuaternios

        #Position data
        self.bit_pos_upd=False
        self.pos_act=[]

        # IMU data
        self.rawIMU=[]
        self.IMUfusion=[]
        self.ang_z_prev=0
        self.lastTime_ang=0

        # PWM data
        self.PWMminP=0.25
        self.PWMmaxP=0.35

        #Variable SS (small space) que sirve para espacios pequeños o de movimiento no libre
        self.SS=False 

        #Márgenes de controladores
        self.angle_margin=5*(math.pi/180) # 5 deg de margen
        self.dmargin=0.1 #10cm de margen
        self.TactAngle=1/17.8 #17.8Hz de update time

    def testWork(self, val):
        self.val = val
        if val:
            print('Clase iniciada correctamente \n')
            print('Todos los modulos se han cargado correctamente')
            print('Valores de kp elegidos: ', self.kp)
            print('Valores de ki elegidos: ', self.ki)
        else:
            print('El digito introducido no verifica los parametros')

    
    def set_dead_zone(self):
        # Dead zone será la zona donde el robot no puede acceder
        # Lo ideal es que se detenga al llegar a esta zona en la máxima proximidad
        # Se construye una matriz cuadrada que definirá una región no accesible
        # A partir de 4 esquinas (o 3)
        # [x0,y0, x1,y1, x2,y2, x3,y3] o [x0,y0, x1,y1, x2,y2]

        if len(self.dead_zone)==6:
            #Zona definida por un triangulo
            #El punto inmovil es el P0
            '''
            m_triang_r1=(dead_zone(3)-dead_zone(1))/(dead_zone(2)-dead_zone(0))
            m_triang_r2=(dead_zone(5)-dead_zone(1))/(dead_zone(4)-dead_zone(0))
            #Recta P1-P2
            m_triang_r3=(dead_zone(5)-dead_zone(3))/(dead_zone(4)-dead_zone(2))
            '''

    def obtener_posicion_angulo(self):
        while not self.bit_pos_upd:
            try:
                self.hedge.dataEvent.wait(1)
                self.hedge.dataEvent.clear()
                #Filtered values
                if (self.hedge.fusionImuUpdated):
                    #data in array [X,Y,Z, QW,QX,QY,QZ, VX,VY,VZ, AX,AY,AZ, timestamp] 
                    self.IMUfusion=self.hedge.imu_fusion()
                    #[AX,AY,AZ, GX,GY,GZ, MX,MY,MZ, timestamp]
                    self.rawIMU=self.hedge.raw_imu()
                    self.bit_pos_upd=True
                    #Stores position+quaternions
            except KeyboardInterrupt:
                self.hedge.stop()  # stop and close serial port
                sys.exit()
        self.bit_pos_upd=False

    def obtener_posicion(self):
        while not self.bit_pos_upd:
            try:
                self.hedge.dataEvent.wait(1)
                self.hedge.dataEvent.clear()
                #Filtered values
                if (self.hedge.positionUpdated):
                    #[AX,AY,AZ, GX,GY,GZ, MX,MY,MZ, timestamp]
                    self.pos_act=self.hedge.position() #hedgeID, X, Y, Z
                    self.bit_pos_upd=True
            except KeyboardInterrupt:
                self.hedge.stop()  # stop and close serial port
                sys.exit()
        self.bit_pos_upd=False

    def obtener_yaw(self, ax, ay, az, gz, timestamp, qw, qx, qy, qz):
        # yaw (z-axis rotation)
        # Falta el filtrado de deriva
        # El codigo siguiente incluye una corrección de deriva
        if not self.way: 
            escalado=131 #suponiendo que ya está en g/s
            dt=timestamp-self.lastTime_ang
            self.lastTime_ang=timestamp
            acc_ang_z=math.atan2(az,math.sqrt(ax**2 + ay**2))
            ang_z=0.98*(self.ang_z_prev+(gz/escalado)*dt)+0.02*acc_ang_z
            self.ang_z_prev=ang_z
            return ang_z
        else:
            #Sin correccion de derivalogSet
            siny_cosp = 2 * (qw * qz + qx * qy)
            cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
            return math.atan2(siny_cosp, cosy_cosp)

    def filtro_angle(self, pos_x, pos_y, ang_compass_in, gyro_z, timestamp):

        #Para que el filtrado sea correcto, esta función debe invocarse varias veces, esto se debe hacer desde el 
        #la funcion principal, para que el bucle de control principal sea la causa de la detencion

        #Por ahora el papel del compass esta desactivado (alta influencia campos magnéticos)

        #Preparacion de parametros
        self.pos_x=pos_x
        self.pos_y=pos_y
        self.ang_compass=ang_compass_in
        self.gyro_z=gyro_z
        self.timestamp=timestamp

        #Carga de entradas
        #1. Angulo posicional
        y_real_ang_pos=math.atan2(self.pos_y_prev-self.pos_y, self.pos_x_prev-self.pos_x)
        self.pos_x_prev=self.pos_x
        self.pos_y_prev=self.pos_y

        """
        #2. Brújula: solo escalado
        offset=0 #se debe observar para alinear ejes
        max_scale=360
        ang_compass=(ang_compass_in-offset)/max_scale
        """
        #3. Giroscopio -> necesaria integracion deg/s -> deg
        #Metodo 2: rectangular (bxh)
        timestampDif=self.timePrev-timestamp
        ang_int= self.ang_int_prev+timestampDif*(self.accAngZprev-self.gyro_z) #ang int es la entrada al KF3
        self.accAngZprev=self.gyro_z
        self.ang_int_prev=ang_int
        self.timePrev=self.timestamp

        #Variables que deben guardarse en memoria, con un subindice para cada filtro:
        #Inicializacion
        #y_pred = 0 #Inicialmente la señal procesada se estima en 0 (irá incrementando)
        #merr = 0 #Inicialmente se estima que el error cuadratico medio MSE es 0
        #a = # bias (nivel de continua) sobre el que se inicializa el filtro
        #sigma_u = 0 #Varianza de la señal de entrada 
        #sigma_n = 0 #Varianza del ruido aplicado a la señal (debe haberlo)

        # KF1: posicional
        # Fase 1: Prediccion
        self.y_pred1 = self.a1*self.y_pred1
        err = y_real_ang_pos-self.y_pred1 # error cometido en la prediccion
        self.m_err1 = (self.a1**2)*self.m_err1 + self.sigma_u1
        K=self.m_err1/(self.sigma_n1+self.m_err1) # calculo de la cte de Kalman
        # Fase 2: Corrección de prediccion
        self.y_pred1 =  self.y_pred1+K*err
        y_est_ang_pos = self.y_pred1 # SALIDA DEL FILTRO
        self.m_err1 = (1-K)*self.m_err1 # actualizacion de MSE
        '''
        #KF2: Compass
        self.y_pred2 = self.a2*self.y_pred2
        err = ang_compass-self.y_pred2 # error cometido en la prediccion
        self.m_err2 = (self.a2**2)*self.m_err2 + self.sigma_u2
        K=self.m_err2/(self.sigma_n2+self.m_err2) # calculo de la cte de Kalman
        # Fase 2: Corrección de prediccion
        self.y_pred2 =  self.y_pred2+K*err
        ang_compass_est = self.y_pred2 # SALIDA DEL FILTRO
        self.m_err2 = (1-K)*self.m_err2 # actualizacion de MSE
        '''
        #KF3: Gyro
        self.y_pred3 = self.a3*self.y_pred3
        err = ang_int-self.y_pred3 # error cometido en la prediccion
        self.m_err3 = (self.a3**2)*self.m_err3 + self.sigma_u3
        K=self.m_err3/(self.sigma_n3+self.m_err3) # calculo de la cte de Kalman
        # Fase 2: Corrección de prediccion
        self.y_pred3 =  self.y_pred3+K*err
        ang_int_est = self.y_pred3 # SALIDA DEL FILTRO
        self.m_err3 = (1-K)*self.m_err3 # actualizacion de MSE

        #Fusión de estimaciones
        #Antes de fusionar se debe haber calculado la covarianza de cada filtro
        #La covarianza se deberia de calcular con la medida real: ej: si se gira 45º
        #la covarianza se calcula respecto a una variable cte de ese valor

        covPos=0.1 #Covarianzas que deben estimarse en base a experimentos
        covComp=0.1
        covGyro=0.1
        '''
        num=y_est_ang_pos/covPos+ang_compass_est/covComp+ang_int_est/covGyro
        den=1/covPos+1/covComp+1/covGyro
        return num/den #Estimación ponderada de los 3 filtrados según covarianzas
        '''

        num=y_est_ang_pos/covPos+ang_int_est/covGyro
        den=1/covPos+1/covGyro
        return [num/den, y_est_ang_pos, ang_int_est] #Estimación ponderada de los 3 filtrados según covarianzas

    def calc_cov(self, in1, in2):
        self.in1=in1
        self.in2=in2
        N=len(in1)
        med1=0
        med2=0
        cov=0
        #calculo de media
        for i in range(0,N-1):
            med1=med1+self.in1(i)
            med2=med2+self.in2(i)
        med1=med1/N
        med2=med2/N
        for i in range(0,N-1):
            cov2=cov2+(self.in1(i)-med1)*(self.in2(i)-med2)
        return cov2/N


    def turn_to(self, angle):
        # Se debe girar el robot hasta la posición dada

        self.angle = angle # Debe estar expresado en (-pi, ++pi)
        PI_ang = classPI(kp=self.kp[0], ki=self.ki[0], lim=self.PWMmaxP)
        
        #Cuando el robot está a 90º está alineado con el eje Y positivo, se gira desde Y

        # Obtencion de angulo inicial
        # self.obtener_posicion_angulo() 
        # valRAW=rawIMU
        # valFusion=IMUfusion 
        # Obtener YAW requiere: ax, ay, az, gz, timestamp
        # RAWimu: [AX,AY,AZ, GX,GY,GZ, MX,MY,MZ, timestamp]
        # FusionIMU: [X,Y,Z, QW,QX,QY,QZ, VX,VY,VZ, AX,AY,AZ, timestamp] 

        # self, ax, ay, az, gz, timestamp, qw, qx, qy, qz
        # angulo_leido=self.obtener_yaw(ax=valFusion[10], ay=valFusion[11],az=valFusion[12],gz=valRAW[5], timestamp=valFusion[13], qw=valFusion[3], qx=valFusion[4], qy=valFusion[5], qz=valFusion[6])


        # Yaw respecto al SR del robot
        #Cte ángulo margen (configurable)
        
        err_PI=self.angle_margin*2 #Para entrar inicialmente al bucle
        angulo_leido=0
        t_prev=0

        if self.debug:
            print('Comienzo del reposicionamiento del robot')
        #Bucle de calculo del PI

        while (self.angle_margin < abs(err_PI)):
            if ((time.time()-t_prev)>=self.TactAngle):
                t_prev=time.time()
                if self.debugTime:
                    inicio=time.time()

                angulo_leido=self.hedge.return_angle() #Return angle ya devuelve el ángulo del eje Y del robot (angulo leido -90)
                #print('Posicion actualizada')

                if self.debugTime:
                    treadAng=time.time()

                #Calculo controlador
                err_PI=self.angle-angulo_leido
                u=PI_ang.PI_SW(err_PI)

                if self.debugTime:
                    tcalcPI=time.time()

                if self.debug:
                    print('Controlador: ', u, 'Angulo objetivo: ', self.angle*(180/math.pi), 'angulo leido: ', angulo_leido*(180/math.pi))

                if self.debugTime:
                    tini_corr=time.time()
                # Corrección de sentido de giro según ángulo menor
                sign=1
                if err_PI>0:
                    if err_PI >= math.pi:
                        sign=-1
                    else:
                        sign=1
                else: #Si el error es <=0
                    if err_PI <= -(math.pi):
                        sign=-1
                    else:
                        sign=1
                if self.debugTime:
                    tfin_corr=time.time()

                #Asignacion a motores

                if not self.SS:
                    self.move.left_motor.value=-u*sign   # Cuando u<1 el robot gira a la izq (giros con incAng>0)
                    self.move.right_motor.value=u*sign     # Cuando u>1 el robot gira a la derecha (giros con incAng<0)
                    if self.logSet == 1:
                        self.data.append([u, u*sign, self.angle, angulo_leido, self.angle_margin])
                else: #No usado  - PWM en tren de pulsos
                    T=0.1
                    self.move.left_motor.value=-u*sign   # Cuando u<1 el robot gira a la izq (giros con incAng>0)
                    self.move.right_motor.value=u*sign     # Cuando u>1 el robot gira a la derecha (giros con incAng<0)
                    if self.logSet == 1:
                        self.data.append([u, u*sign, self.angle, angulo_leido, self.angle_margin])
                    time.sleep(T/2)
                    self.move.left_motor.value=0   # Cuando u<1 el robot gira a la izq (giros con incAng>0)
                    self.move.right_motor.value=0     # Cuando u>1 el robot gira a la derecha (giros con incAng<0)
                    if self.logSet == 1:
                        self.data.append([0, 0, self.angle, angulo_leido, self.angle_margin])       
                    time.sleep(T/2)
                
                if self.debugTime:
                    final=time.time()
                    print('t de ejecucion de iteracion completa= ', (final-inicio)*10**6)
                    print('t de lectura de angulo: ', (treadAng-inicio)*10**6)
                    print('t de calculo de controlador: ', (tcalcPI-treadAng)*10**6)
                    print('t de correccion de signo: ', (tfin_corr-tini_corr)*10**6)
                    print('t de asig puente H: ', (final-tfin_corr)*10**6)

        #Detencion al alcanzar angulo
        self.move.left_motor.value=0
        self.move.right_motor.value=0 # Tras girar se detienen los motores

        #Si el log está activado se guardan los datos
        if self.logSet == 1:
            csvfile = open("/home/jetbot/Desktop/JetBot_Balizas/jetbotMotion/logAng.csv",'w', newline='')
            with csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self.data)
            print('Se ha finalizado la captura de datos')


    def go_to(self, x, y):
        self.x = x
        self.y = y

        # Posicion inicial
        self.obtener_posicion() 
        if self.debug:
            print('Posicion RAW ',self.pos_act)

        # pos_act: [hedgeID, X, Y....]
        posicion_leida=[self.pos_act[1], self.pos_act[2]]
        dify=self.y-posicion_leida[1]
        difx=self.x-posicion_leida[0]
        initial_rot=math.atan2(dify,difx)


        if initial_rot <0:
            initial_rot=initial_rot + 2*math.pi #Siempre se trabaja con ángulo positivo (+360º)

        self.turn_to(initial_rot) #giro inicial 
        # initial_rot guarda el ángulo de orientación respecto al SR balizas
        # Tras el primer giro se comienza el comando comjunto de dirección+angulo

        # Distancia inicial
        #angle_margin=3*(math.pi/180) # 3 deg de margen
        distance=math.sqrt(dify**2+difx**2)
        # La parte de corrección de ángulo se ejecutará siempre

        # Instanciación del doble PI
        PIang=classPI(kp=self.kp[0], ki=self.ki[0], lim=self.PWMmaxP)
        PIpos=classPI(kp=self.kp[1], ki=self.ki[1], lim=self.PWMmaxP)
        sat=False

        # Bucle de cálculo de PIs
        while (distance > self.dmargin):

            # Calculo de posicion y angulo iterativos
            self.obtener_posicion() 

            self.hedge.dataEvent.wait(1)
            self.hedge.dataEvent.clear()
            while not self.hedge.positionUpdated:
                angulo_leido=0
            angulo_leido=self.hedge.return_angle() #Return angle ya devuelve el ángulo del eje Y del robot (angulo leido -90)
            
            posicion_leida=[self.pos_act[1], self.pos_act[2]]
            dify=self.y-posicion_leida[1]
            difx=self.x-posicion_leida[0]
            distance=math.sqrt(dify**2+difx**2)
            rot=math.atan2(dify,difx)
            if rot <0:
                rot=rot + 2*math.pi #Siempre se trabaja con ángulo positivo (+360º)

            # Doble PI (posicion+angulo)
            err_ang=rot-angulo_leido
            err_pos=distance-self.dmargin
            uAng=PIang.PI_SW_ext(err=err_ang, sat_act=sat) #PIs con control de saturacion externo
            uPos=PIpos.PI_SW_ext(err=err_pos, sat_act=sat)
        
            # Salida PWM controlada

            # Corrección de sentido de giro según ángulo menor
            sign=1
            if err_ang>0:
                if err_ang >= math.pi:
                    sign=-1
                else:
                    sign=1
            else: #Si el error es <=0
                if err_ang <= -(math.pi):
                    sign=-1
                else:
                    sign=1

            uIzq = uPos - uAng*sign
            uDer = uPos + uAng*sign
            
            # Anti windup + control de saturacion
            if uIzq>self.PWMmaxP:
                uIzq=self.PWMmaxP
                sat=True
            elif uIzq<-self.PWMmaxP:
                uIzq=-self.PWMmaxP
                sat=True
            else:
                if sat:
                    sat=False

            if uDer>=self.PWMmaxP:
                uDer=self.PWMmaxP
                sat=True
            elif uDer<=-self.PWMmaxP:
                uDer=-self.PWMmaxP
                sat=True
            else:
                if sat:
                    sat=False

            # Asignacion a motores

            if not self.SS:
                self.move.left_motor.value=uIzq
                self.move.right_motor.value=uDer
                if self.logSet == 2:
                    self.data.append([uIzq, uDer, rot, angulo_leido, distance, self.dmargin, posicion_leida[0], posicion_leida[1]])
            else: #En tren de pulsos, no usado actualmente
                T=0.1
                self.move.left_motor.value=uIzq   # Cuando u<1 el robot gira a la izq (giros con incAng>0)
                self.move.right_motor.value=uDer     # Cuando u>1 el robot gira a la derecha (giros con incAng<0)
                if self.logSet == 2:
                    self.data.append([uIzq, uDer, rot, angulo_leido, distance, self.dmargin, posicion_leida[0], posicion_leida[1]])
                time.sleep(2*T/3)
                self.move.left_motor.value=0   # Cuando u<1 el robot gira a la izq (giros con incAng>0)
                self.move.right_motor.value=0     # Cuando u>1 el robot gira a la derecha (giros con incAng<0)
                if self.logSet == 2:
                    self.data.append([0, 0, rot, angulo_leido, distance, self.dmargin, posicion_leida[0],posicion_leida[1]])
                time.sleep(T/3)
            if self.debug:
                print('Motor iz [u]: ', uIzq, 'Motor der [u]: ', uDer)
            #time.sleep(T) # Ajuste de t de muestreo

        #Detencion de motores
        self.move.left_motor.value=0
        self.move.right_motor.value=0 # Tras girar se detienen los motores

        #Si el log está activado se guardan los datos
        if self.logSet == 2:
            csvfile = open("/home/jetbot/Desktop/JetBot_Balizas/jetbotMotion/logAngPos.csv",'w', newline='')
            with csvfile:
                writer = csv.writer(csvfile)
                writer.writerows(self.data)
            print('Se ha finalizado la captura de datos')


    
    def take_sample(self):
        #Se tomará una foto con la cámara en la ubicación alcanzada
        from jupyter_clickable_image_widget import ClickableImageWidget
        DATASET_DIR = 'dataset_xy'

        # we have this "try/except" statement because these next functions can throw an error if the directories exist already
        try:
            os.makedirs(DATASET_DIR)
        except FileExistsError:
            print('Directories not created because they already exist')

        camera = Camera()

        # create image preview
        camera_widget = ClickableImageWidget(width=camera.width, height=camera.height)
        snapshot_widget = ipywidgets.Image(width=camera.width, height=camera.height)
        traitlets.dlink((camera, 'value'), (camera_widget, 'value'), transform=bgr8_to_jpeg)

        # create widgets
        count_widget = ipywidgets.IntText(description='count')
        # manually update counts at initialization
        count_widget.value = len(glob.glob(os.path.join(DATASET_DIR, '*.jpg')))

        def save_snapshot(_, content, msg):
            if content['event'] == 'click':
                data = content['eventData']
                x = data['offsetX']
                y = data['offsetY']
                
                # save to disk
                #dataset.save_entry(category_widget.value, camera.value, x, y)
                uuid = 'xy_%03d_%03d_%s' % (x, y, uuid1())
                image_path = os.path.join(DATASET_DIR, uuid + '.jpg')
                with open(image_path, 'wb') as f:
                    f.write(camera_widget.value)
                
                # display saved snapshot
                snapshot = camera.value.copy()
                snapshot = cv2.circle(snapshot, (x, y), 8, (0, 255, 0), 3)
                snapshot_widget.value = bgr8_to_jpeg(snapshot)
                count_widget.value = len(glob.glob(os.path.join(DATASET_DIR, '*.jpg')))
                
        camera_widget.on_msg(save_snapshot)

        data_collection_widget = ipywidgets.VBox([
            ipywidgets.HBox([camera_widget, snapshot_widget]),
            count_widget
        ])

        display(data_collection_widget)

    def stop(self):
        self.hedge.stop()  # stop and close serial port
        sys.exit()
        
