from CVInterface import CVInterface
from time import sleep
cv = CVInterface(1)
boundrypixel= cv.get_course_boundary()
while True:
    cv.get_robot_position_and_rotation()
    cv.get_ball_positions()


    
