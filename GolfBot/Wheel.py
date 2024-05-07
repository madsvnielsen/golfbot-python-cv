from ev3dev2.wheel import Wheel
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank

tire = Wheel(68.8, 36)
leftWheel = LargeMotor(OUTPUT_A)
rightWheel = LargeMotor(OUTPUT_B)
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
disFromrobt = 10  # should be an input with actual distance
distanceToBall = disFromrobt*10
# Caluclate wheel rotaions needed to move distance to ball
rotations_needed = distanceToBall / tire.circumference_mm

tank_drive.on_for_rotations(SpeedPercent(
    50), SpeedPercent(75), rotations_needed)

leftWheel.on_for_rotations(SpeedPercent(50), rotations_needed)
rightWheel.on_for_rotations(SpeedPercent(50), rotations_needed)
