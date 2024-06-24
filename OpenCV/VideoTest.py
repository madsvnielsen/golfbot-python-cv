import math

from OpenCV.CVInterface import CVInterface
import time
import numpy as np
from OpenCV.RobotController import getRobotPosition, create_navigation_route, acquireTargetBall
cv = CVInterface(1)
boundrypixel= cv.get_course_boundary()


cv.get_goals()

def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)


def convert_to_pixel(normalized_coords, square_bottom_left, square_top_right):
    square_dimensions_cm = (180, 120)
    normalized_x, normalized_y = (normalized_coords[0], normalized_coords[1])
    square_width_pixels = square_top_right[0] - square_bottom_left[0]
    square_height_pixels = square_top_right[1] - square_bottom_left[1]

    # Calculate the size of one unit increment in centimeters
    one_unit_increment_cm_width = square_dimensions_cm[0] / square_width_pixels
    one_unit_increment_cm_height = square_dimensions_cm[1] / \
        square_height_pixels

    # Convert normalized coordinates back to relative pixel coordinates
    relative_x = normalized_x / one_unit_increment_cm_width
    relative_y = normalized_y / one_unit_increment_cm_height

    # Add the bottom-left corner of the square to get the absolute pixel coordinates
    pixel_x = relative_x + square_bottom_left[0]
    pixel_y = relative_y + square_bottom_left[1]

    return pixel_x, pixel_y



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


def correct_robot_coordinate_pixels(position, boundrypixel, cv):
    position_normal = convert_to_normalized(position, boundrypixel["bottom_left"], boundrypixel["top_right"])
    center_normal = convert_to_normalized(cv.get_center(), boundrypixel["bottom_left"], boundrypixel["top_right"])
    distance_from_center = euclidean_distance(position_normal, center_normal)
    distance_error = find_error_in_plane(distance_from_center)
    print(center_normal)

    position_vector = np.array(position_normal)
    center_vector = np.array(center_normal)

    direction_vector = center_vector - position_vector
    direction_vector_normalized = direction_vector / np.linalg.norm(direction_vector)
    
    # Scale the direction vector by the error
    offset_vector = direction_vector_normalized * distance_error
    
    # Calculate the new robot center by adding the offset vector
    new_position =  tuple(position_normal + offset_vector)

    position_fixed_pixels = convert_to_pixel(new_position , boundrypixel["bottom_left"], boundrypixel["top_right"])
   

    print(cv.get_center())

    return new_position

def loop():
    i = 0
    while True:
        i += 1
        if i == 30:
            cv.update_grid()
            i = 0
        ball_positions = cv.get_ball_positions()
        robot_position_pixels = cv.get_robot_position_and_rotation()["origin"]
        if robot_position_pixels == None: continue
        robot_position_fixed = correct_robot_coordinate_pixels(robot_position_pixels, boundrypixel, cv)

        
    

        '''
        if len(ball_positions) >= 2:
            ballsNormalized1 = convert_to_normalized(ball_positions[0], boundrypixel["bottom_left"], boundrypixel["top_right"])
            ballsNormalized2 = convert_to_normalized(ball_positions[1], boundrypixel["bottom_left"], boundrypixel["top_right"])
            distance = euclidean_distance(ballsNormalized1, ballsNormalized2)
            print(f"The distance between the first two balls is: {distance}")
        else:
            print("Less than two balls were detected.")
        '''
        


#cv.initialize_grid((int(1920/16), int(1080/16)))






def loop2():
    cv = CVInterface(1)
    boundrypixel = cv.get_course_boundary()
    cv.initialize_grid((int(1920/32), int(1080/32)))

    navigation_waypoints = []
    actual_target = None

    assumed_balls_in_mouth = 0
    pickup_threshold = 5
    focus_target_threshold = 30
    target_is_ball = False

    while True:
        cv.update_grid()
        cv.get_goals()
        robotcenter, robotfront, robotDir = getRobotPosition(cv, boundrypixel)
        
        if robotcenter == None:
            continue
        acquireTargetBall(cv, boundrypixel)
        if len(navigation_waypoints) == 0:
            active_target, target_pixels = acquireTargetBall(cv, boundrypixel)
            navigation_waypoints, target, is_ball = create_navigation_route(cv, boundrypixel, robotfront, active_target, target_pixels)
            target_is_ball = is_ball
            actual_target = active_target
            print("Waypoints", navigation_waypoints)
            continue
        
        vector_to_actual_target = (actual_target[0] - robotcenter[0], actual_target[1] - robotcenter[1])
        distance_to_actual_target = euclidean_distance(actual_target, robotfront)

        if abs(distance_to_actual_target < focus_target_threshold):
            active_target = actual_target

        active_target = navigation_waypoints[0]
        vector_to_active_target = (active_target[0] - robotcenter[0], active_target[1] - robotcenter[1])
        distance_to_target = euclidean_distance(active_target, robotfront)
        origin_distance_to_target = euclidean_distance(active_target, robotcenter)
        ball_positions = cv.get_ball_positions()

       


    

def start_video_test():
    loop2()