#!/usr/bin/env python3

from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.motor import MediumMotor, OUTPUT_C
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds


# This program requires LEGO EV3 MicroPython v2.0 or higher.½
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
golfBot = MoveTank(OUTPUT_A, OUTPUT_B)
golfBot.gyro = GyroSensor()
ballPicker = MediumMotor(OUTPUT_C)

# Write your program here.


    
