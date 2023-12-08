#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBaseS
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

right_motor = Motor(Port.A)
left_motor = Motor(Port.D)
CSenRight = ColorSensor(Port.S3)
CSenLeft = ColorSensor(Port.S2)
UltSen = UltrasonicSensor(Port.S1)

girar_garra = Motor(Port.B)
fechar_garra = Motor(Port.C)

MOV = DriveBase(left_motor, right_motor, wheel_diameter=40, axle_track=135)

Kp = 3.5  # const P
Ki = 0.1  # const I
Kd = 0.1  # const D
speed = 70 # Velocidade em mm/s

# tamanho dos obstaculos
Com = 100/2
Lar = 100/2

I = 0
P_ant = 0

def PID(I, Kp, Ki, Kd, speed, MOV, P_ant):
    I = I + P
    D = P - P_ant

    saida = Kp * P + Ki * I + Kd * D

    MOV.drive(speed, saida)

    P_ant = P

def VirarRight(MOV, PID):
    MOV.stop()
    MOV.turn(90)
    MOV.straight(50)

def VirarLeft(MOV):
    MOV.stop()
    MOV.turn(-90)
    MOV.straight(50)

def OBS_Right(MOV, Com, Lar):
    MOV.stop()
    MOV.turn(90)
    MOV.straight(Lar)
    MOV.turn(-90)
    MOV.straight(Com)
    MOV.turn(-90)
    MOV.straight(Lar)
    MOV.turn(90)
    
def Resgate(MOV, UltSen, girar_garra, fechar_garra):
    if UltSen.distance() >= 100:
        MOV.drive(70, 0)
    else:
        MOV.stop()
        girar_garra.run_target(50, -180, then=Stop.HOLD, wait=True)
        MOV.straight(100)
        fechar_garra.run_time(50, 3, then=Stop.HOLD, wait=True)
        girar_garra.run_target(50, 180, then=Stop.HOLD, wait=True)
        MOV.straight(-50)
        MOV.turn(90)
        
while True:
    RcorLeft = CSenRight.reflection()
    RcorRight = CSenRight.reflection()

    P = RcorLeft - RcorRight

    if UltSen.distance() >= 100:
        OBS_Right(MOV, Com, Lar)
        PID(I, Kp, Ki, Kd, speed, MOV, P_ant)
    elif CSenRight.color() == color.GREEN and CSenLeft.color() == color.GREEN:
        PID(I, Kp, Ki, Kd, speed, MOV, P_ant)
    elif CSenLeft.color() == color.GREEN:
        VirarLeft(MOV)
        PID(I, Kp, Ki, Kd, speed, MOV, P_ant)
    elif CSenRight.color() == color.GREEN:
        VirarRight(MOV)
        PID(I, Kp, Ki, Kd, speed, MOV, P_ant)
    else:
        PID(I, Kp, Ki, Kd, speed, MOV, P_ant)
