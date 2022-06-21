#Codigo que alberga un controlador PI implementado mediante backward Euler
#Incluye control de saturacion y anti-windup

# Implementacion PI: u=p+i -> Backward Euler
# p(k)=Kp*(c*ref(k)-med(k))
# i(k)=i(k-1)+(kp/Ti)*T*err(k-1)
# donde T es el periodo de muestreo

class classPI():
    def __init__(self, kp, ki, lim):
        self.kp=kp
        self.ki=ki
        self.err_prev=0
        self.i_prev=0
        self.wnd_i=False
        self.lim=lim

    def PI_SW(self,err):
        self.err=err
        p=self.kp*self.err

        #Proteccion anti-windup + calculo termino i
        if not self.wnd_i:
            i=self.i_prev+self.ki*self.err_prev
        else:
            i=self.i_prev
        self.err_prev=self.err
        self.i_prev=i # Actualizacion de i_prev

        #Salida del controlador
        u=p+i

        if u>self.lim: # Saturacion de la salida + antiwindup
            u=self.lim
            self.wnd_i=True #Pausa del cálculo de i
        elif u<-self.lim:
            u=-self.lim
            self.wnd_i=True
        else:
            if self.wnd_i:
                self.wnd_i = False #Puede seguir calculandose i

        return u

    def PI_SW_ext(self,err,sat_act):
        #PI con control de saturacion externo

        self.err=err
        self.wnd_i=sat_act
        p=self.kp*self.err

        #Proteccion anti-windup + calculo termino i
        if not self.wnd_i:
            i=self.i_prev+self.ki*self.err_prev
        else:
            i=self.i_prev
        self.err_ext=self.err
        self.i_prev=i # Actualizacion de i_prev

        #Salida del controlador
        u=p+i

        return u
        