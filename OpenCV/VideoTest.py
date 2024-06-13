import math

from CVInterface import CVInterface
import time

cv = CVInterface(1)
boundrypixel= cv.get_course_boundary()
cv.get_goals()
def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)


while True:
    ball_positions = cv.get_ball_positions()
    if len(ball_positions) >= 2:
        distance = euclidean_distance(ball_positions[0], ball_positions[1])
        print(f"The distance between the first two balls is: {distance}")
    else:
        print("Less than two balls were detected.")
    time.sleep(1)




    
