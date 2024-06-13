import math

from CVInterface import CVInterface
import time

cv = CVInterface(1)
boundrypixel= cv.get_course_boundary()
cv.get_goals()
def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

def convert_to_normalized(pixel_coords, square_bottom_left, square_top_right):

    square_dimensions_cm = (180, 120)
    pixel_x, pixel_y = (pixel_coords[0], pixel_coords[1])
    square_width_pixels = square_top_right[0] - square_bottom_left[0]
    square_height_pixels = square_top_right[1] - square_bottom_left[1]

    # Calculate the size of one unit increment in centimeters
    one_unit_increment_cm_width = square_dimensions_cm[0] / square_width_pixels
    one_unit_increment_cm_height = square_dimensions_cm[1] / square_height_pixels

    # Subtract the bottom-left corner of the square from the pixel coordinates
    relative_x = pixel_x - square_bottom_left[0]
    relative_y = pixel_y - square_bottom_left[1]

    # Normalize the coordinates
    normalized_x = relative_x * one_unit_increment_cm_width
    normalized_y = relative_y * one_unit_increment_cm_height

    return normalized_x, normalized_y

def find_error_in_plane(robotDistanceFromCenter):
    cameraHeight = 155
    robotHeight = 24
    angleA = math.atan(cameraHeight / robotDistanceFromCenter)
    return robotHeight/math.tan(angleA)




while True:
    ball_positions = cv.get_ball_positions()


    if len(ball_positions) >= 2:
        ballsNormalized1 = convert_to_normalized(ball_positions[0], boundrypixel["bottom_left"], boundrypixel["top_right"])
        ballsNormalized2 = convert_to_normalized(ball_positions[1], boundrypixel["bottom_left"], boundrypixel["top_right"])
        distance = euclidean_distance(ballsNormalized1, ballsNormalized2)
        print(f"The distance between the first two balls is: {distance}")
    else:
        print("Less than two balls were detected.")
    time.sleep(1)




    
