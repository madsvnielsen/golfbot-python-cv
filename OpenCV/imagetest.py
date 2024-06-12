from CVInterface import CVInterface
from time import sleep
cv = CVInterface("../Images/testim3.jpg")
cv.get_robot_position_and_rotation()
cv.get_ball_positions()
boundrypixel= cv.get_course_boundary()

    
