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

motorD = Motor.Port(A)
motorE = Motor.Port(B)
senseD = ColorSensor.Port(S1)
senseE = ColorSensor.Port(S2)

kp = 1
dd = 0

# Write your program here.
ev3.speaker.beep()

while True:
    Cright = senseD.reflection()
    Cleft = senseE.reflection()
    
    dr = Cright - Cleft
    error = kp * (dd - dr)
    
    Mleft = 50 + error
    Mright = 50 - error
    
    motorD.run(Mright)
    motorE.run(Mleft)

