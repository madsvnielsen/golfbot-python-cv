#!/usr/bin/env python3
# IMPORTANT THAT THE MOROTRS AND OTHER DEVICES ARE CONNECTED CORRECTLY
# IMPORTANT THAT THE MOROTRS AND OTHER DEVICES ARE CONNECTED CORRECTLY
# IMPORTANT THAT THE MOROTRS AND OTHER DEVICES ARE CONNECTED CORRECTLY
# IMPORTANT THAT THE MOROTRS AND OTHER DEVICES ARE CONNECTED CORRECTLY
# IMPORTANT THAT THE MOROTRS AND OTHER DEVICES ARE CONNECTED CORRECTLY
from time import sleep

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.motor import MediumMotor, OUTPUT_C
from ev3dev2.sensor.lego import GyroSensor, INTPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds


# This program requires LEGO EV3 MicroPython v2.0 or higher.½
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
golfBot = MoveTank(OUTPUT_A, OUTPUT_B)
golfBot.gyro = GyroSensor(INTPUT_1)
ballPicker = MediumMotor(OUTPUT_C)

# Write your program here.
def testDrive():
    golfBot.on_for_seconds(200, 200, 10, True, True)
    golfBot.turn_degrees(speed=SpeedPercent(5), target_angle=90)
    
x = 0
while x<3:
    if(x==0):
        ballPicker.run_forever
    golfBot.gyro.calibrate()    
    testDrive()
    x += 1
    if(x == 3):
        ballPicker.stop
        golfBot.on_for_seconds(200, 200, 10, True, True)
        break

    


