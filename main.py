#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# Create your objects here.
ev3 = EV3Brick()

right_motor = motor(Port.A)
left_motor = motor(Port.D)
CSenRight = ColorSensor(Port.S1)
CSenLeft = ColorSensor(Port.S2)

MOV = DriveBase(left_motor, right_motor, wheel_diameter=40, axle_track=135)

# Write your program here.

Kp = 1.5  # const P
Ki = 0.1  # const I
Kd = 0.1  # const D
speed = 250 # Velocidade em mm/s

P_des = 0

I = 0
P_ant = 0

def PID(dif):
    
    I = I + P
    D = P - P_ant

    saida = Kp * P + Ki * I + Kd * D

    MOV.drive(speed, saida)

    P_ant = P

while True:
    RcorLeft = CSenRight.reflection()
    RcorRight = CSenRight.reflection()

    P = RcorLeft - RcorRight
    if CSenRight.rgb() == (0, 100, 0) and CSenLeft.rgb() == (0, 100, 0):
        PID(P)
    elif CSenLeft.rgb() == (0, 100, 0):
        VirarLeft()
    elif CSenRight.rgb() == (0, 100, 0):
        VirarRight()
    else:
        PID(P)
