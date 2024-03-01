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


# Write your program here.
ev3.speaker.beep()
wheel_diameter = 68
alex_track = 210


ball_picker = Motor(Port.D, gears = None, positive_direction = Direction.CLOCKWISE)
left_motor = Motor(Port.B, gears = None)
right_motor = Motor(Port.A, gears = None)
robot = DriveBase(left_motor= left_motor, right_motor= right_motor, wheel_diameter= wheel_diameter, axle_track = alex_track)
robot.settings(straight_speed = 200, straight_acceleration = 50)

def pickBalls():
    ball_picker.run(100)


def shootBalls():
    ball_picker.hold
    ball_picker.run(-100)
    

def testDrive():
    robot.straight(100)
    robot.angle(180)
    robot.straight(100)


pickBalls()
x = 0
while x<2:
    testDrive()
    x += 1
shootBalls()






