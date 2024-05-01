import math
import cv2

import main
import Server1
from CVInterface import CVInterface

def start():
    cv =CVInterface(0)

    boundrypixel= cv.get_course_boundary()

    ballspixelcords= cv.get_ball_positions()

    def convert_to_normalized(pixel_coords, square_bottom_left):
        square_dimensions_cm=(180,120)
        pixel_x, pixel_y = pixel_coords
        square_x, square_y = square_bottom_left
        square_width_cm, square_height_cm = square_dimensions_cm

        # Calculate the size of one unit increment in centimeters
        one_unit_increment_cm_width = square_width_cm / (pixel_x - square_x)
        one_unit_increment_cm_height = square_height_cm / (pixel_y - square_y)

        # Subtract the bottom-left corner of the square from the pixel coordinates
        relative_x = pixel_x - square_x
        relative_y = pixel_y - square_y

        # Normalize the coordinates
        normalized_x = relative_x * one_unit_increment_cm_width
        normalized_y = relative_y * one_unit_increment_cm_height

        return normalized_x, normalized_y

    balls_normalized = []
    for coords in ballspixelcords:
        normalized_coords = convert_to_normalized(coords, boundrypixel["left"])
        balls_normalized.append(normalized_coords)

    robotpixel=cv.get_robot_position_and_rotation()
    robotcenter= convert_to_normalized(robotpixel[0],boundrypixel["left"])
    robotfront= convert_to_normalized(robotpixel[1],boundrypixel["left"])

    def find_robot_direction_and_angle(center_coords, front_coords):
        # Calculate the vector representing the direction the robot is facing
        direction = (front_coords[0] - center_coords[0], front_coords[1] - center_coords[1])

        # Calculate the angle between the direction vector and the x-axis
        angle = math.degrees(math.atan2(direction[1], direction[0]))
        if angle < 0:
            angle += 360  # Ensure the angle is positive

        return direction, angle
    robotDir= find_robot_direction_and_angle(robotcenter,robotfront)
    # Function to calculate Euclidean distance between two points
    def euclidean_distance(coord1, coord2):
        return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

    def angle_between(v1, v2):
        dot_product = sum((a * b) for a, b in zip(v1, v2))
        magnitude_v1 = math.sqrt(sum(a ** 2 for a in v1))
        magnitude_v2 = math.sqrt(sum(b ** 2 for b in v2))
        return math.acos(dot_product / (magnitude_v1 * magnitude_v2))

    # Calculate distances
    distances = [euclidean_distance(robotcenter, coord) for coord in balls_normalized]

    # Find the index of the coordinate with the smallest distance
    closest_index = distances.index(min(distances))

    # Get the closest coordinate
    closest_coordinate = balls_normalized[closest_index]
    vector_to_closest_coordinate = (closest_coordinate[0] - robotcenter[0], closest_coordinate[1] - robotcenter[1])

    #calculate angle to turn
    angle_to_turn = angle_between((math.cos(robotDir[1]), math.sin(robotDir[1])), vector_to_closest_coordinate)
    if angle_to_turn > 180:
        Server1.recvCommand("left")
    elif 0 < angle_to_turn < 180:
        Server1.recvCommand("right")
    elif angle_to_turn==0:
        Server1.recvCommand("forward")
    start_generator=main.start()

    for keypoints in start_generator:
        balls = keypoints
        print('cordinates'+balls)

    robot=cv2.KeyPoint(100, 200, _size=10, _angle=0, _response=0, _octave=0, _class_id=-1)


