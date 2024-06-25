import math
from time import sleep, time

import numpy as np

from OpenCV.CVInterface import CVInterface
from OpenCV.Server1 import Server

def convert_to_normalized(pixel_coords, square_bottom_left, square_top_right):

    square_dimensions_cm = (180, 120)
    pixel_x, pixel_y = (pixel_coords[0], pixel_coords[1])
    square_width_pixels = square_top_right[0] - square_bottom_left[0]
    square_height_pixels = square_top_right[1] - square_bottom_left[1]

    # Calculate the size of one unit increment in centimeters
    one_unit_increment_cm_width = square_dimensions_cm[0] / square_width_pixels
    one_unit_increment_cm_height = square_dimensions_cm[1] / \
        square_height_pixels

    # Subtract the bottom-left corner of the square from the pixel coordinates
    relative_x = pixel_x - square_bottom_left[0]
    relative_y = pixel_y - square_bottom_left[1]

    # Normalize the coordinates
    normalized_x = relative_x * one_unit_increment_cm_width
    normalized_y = relative_y * one_unit_increment_cm_height

    return normalized_x, normalized_y


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

def find_center_between_points(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    center_x = (x1 + x2) / 2
    center_y = (y1 + y2) / 2
    return (center_x, center_y)

def find_robot_direction_and_angle(center_coords, front_coords):
    # Calculate the vector representing the direction the robot is facing
    direction = (front_coords[0] - center_coords[0],
                 front_coords[1] - center_coords[1])
    '''
        # Calculate the angle between the direction vector and the x-axis
        angle = math.degrees(math.atan2(direction[1], direction[0]))
        if angle < 0:
            angle += 360  # Ensure the angle is positive
        '''
    return direction


def find_error_in_plane(robotDistanceFromCenter):
    cameraHeight = 170
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

    #position_fixed_pixels = convert_to_pixel(new_position , boundrypixel["bottom_left"], boundrypixel["top_right"])

    return new_position

# Function to calculate Euclidean distance between two points
def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)


def angle_between(v1, v2):
    radians = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
    angle = math.degrees(radians)
    if angle < -180:
        angle += 360
    elif angle > 180:
        angle -= 360
    return angle


def getRobotPosition(cv, boundrypixel):
    robotpixel = cv.get_robot_position_and_rotation()

    if (robotpixel["origin"] == None or robotpixel["direction"] == None):
        return None, None, None
    robotcenter = correct_robot_coordinate_pixels(robotpixel["origin"], boundrypixel, cv)
    robotfront = correct_robot_coordinate_pixels(robotpixel["direction"], boundrypixel, cv)
    

    cv.projection= {
        "r_center" : convert_to_pixel(robotcenter, boundrypixel["bottom_left"], boundrypixel["top_right"]),
        "r_front" : convert_to_pixel(robotfront, boundrypixel["bottom_left"], boundrypixel["top_right"]),
    }

    return robotcenter, robotfront, find_robot_direction_and_angle(robotcenter, robotfront)


# Function to project points using homography matrix
def project_points_to_ground_plane(points, H):
    points = np.array(points)
    if points.ndim == 1:
        # Convert to 2D array if it's a single point
        points = points[np.newaxis, :]

    # Add ones to the points (homogeneous coordinates)
    points = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)

    # Transform points using homography matrix
    projected_points = H.dot(points.T).T

    # Normalize the points
    projected_points = projected_points[:,
        :2] / projected_points[:, 2, np.newaxis]

    return projected_points


def create_navigation_route(cv, boundrypixel, robotcenter, active_target, target_pixels,notBall = False):
    
    cv.target_pos = target_pixels
    grid = cv.get_grid()
    if active_target == None:
        return [], None, False
    
    robotcenter_pixels = convert_to_pixel(robotcenter,boundrypixel["bottom_left"], boundrypixel["top_right"])
    target_grid_cell = grid.get_position_from_pixel(target_pixels)
    source_grid_cell = grid.get_position_from_pixel(robotcenter_pixels)
    
    navigation_cell_coordinates = grid.navigate([target_grid_cell], [source_grid_cell])
    if navigation_cell_coordinates is None:
        return [], None, False
    print(navigation_cell_coordinates)
    path_in_pixels = [(cell[1], cell[0]) for cell in [grid.get_pixel_from_position(a) for a in navigation_cell_coordinates]][1::3]
    cv.waypoints = path_in_pixels
    path_in_normalized = [convert_to_normalized(cell,boundrypixel["bottom_left"], boundrypixel["top_right"]) for cell in path_in_pixels]
    path_in_normalized.append(active_target)
    min_distance_between_waypoints = 10

    if len(path_in_normalized) >= 2 and euclidean_distance(active_target, path_in_normalized[-2]) < min_distance_between_waypoints:
        del path_in_normalized[-2]
    return path_in_normalized, active_target, True if not notBall else False


def start_robot_controller():
    cv = CVInterface(1)
    server = Server()
    boundrypixel = cv.get_course_boundary()
    cv.get_goals()
    cv.initialize_grid((int(1920/32), int(1080/32)))

    navigation_waypoints = []
    target_position = None

    assumed_balls_in_mouth = 0
    pickup_threshold = 5
    focus_target_threshold = 30
    target_is_ball = False

    server.send_key_input("start")

    last_target = None
    max_target_retries = 10
    current_target_tries = 0
    slow = False
    while True:
        
        robotcenter, robotfront, robotDir = getRobotPosition(cv, boundrypixel)
        if robotcenter == None:
            continue

        if len(navigation_waypoints) == 0:
            active_target, target_pixels = acquireTargetBall(cv, boundrypixel)
            navigation_waypoints, target, is_ball = create_navigation_route(cv, boundrypixel, robotfront, active_target, target_pixels)
            target_position = target
            target_is_ball = is_ball
            current_target_tries = 0
            if navigation_waypoints == None:
                navigation_waypoints = []

            cv.update_grid()
            continue
        
        
        
        distance_to_actual_target = euclidean_distance(target_position, robotfront)


        active_target = navigation_waypoints[0]
        focus_mode = False

        
        '''
        if abs(distance_to_actual_target < focus_target_threshold):
            focus_mode = True
            active_target = target_position
        '''

        cv.target_pos = convert_to_pixel(active_target, boundrypixel["bottom_left"], boundrypixel["top_right"])

        robotmid = find_center_between_points(robotcenter, robotfront)

        vector_to_active_target = (active_target[0] - robotcenter[0], active_target[1] - robotcenter[1])

        distance_to_target = euclidean_distance(active_target, robotfront)
        origin_distance_to_target = euclidean_distance(active_target, robotcenter)
        mid_distance_to_target = euclidean_distance(active_target, robotmid)
        

        

        ## See if robot is at target destination
        robot_points = [distance_to_target, mid_distance_to_target,  origin_distance_to_target] if target_is_ball else [origin_distance_to_target]
        if min(robot_points) < pickup_threshold:
            
            del navigation_waypoints[0]
            current_target_tries = 0
            if len(navigation_waypoints) == 0:
                #server.send_key_input("forward 0.33")
                #sleep(1)
                if target_is_ball:
                    assumed_balls_in_mouth += 1
                    slow = False
                    server.send_key_input("back")
                    sleep(3)
            
            if assumed_balls_in_mouth > 4 or (len(cv.get_ball_positions()) == 0 and len(navigation_waypoints) == 0):
                assumed_balls_in_mouth = 0
                deposit_balls(cv, boundrypixel, server)
            last_target = active_target
            continue

        power_through = False #len(navigation_waypoints) == 1 and cv.get_grid().get_cell_at_position(cv.target_pos).status != "blocked"

        # calculate angle to turn
        move_robot_towards_target(server, robotDir, vector_to_active_target, distance_to_target, power_through, slow)

        if last_target is not None and target_is_ball:
            if last_target == active_target and not slow:
                current_target_tries += 1
            if current_target_tries > max_target_retries:
                server.send_key_input("back")
                navigation_waypoints = []
                current_target_tries = 0
                continue
        last_target = active_target

def move_robot_towards_target(server, robotDir, vector_to_active_target, distance_to_target, power_through, slow):
    angle_to_turn = angle_between((robotDir), vector_to_active_target)
    turn_in_seconds = abs((angle_to_turn/90)*1.5)
    if turn_in_seconds < 0.1:
        turn_in_seconds = 0.1

    angle_threshold = 1.5

    if angle_threshold < angle_to_turn < 180:
        server.send_key_input("mleft " + str(turn_in_seconds) + " ")
    elif -angle_threshold > angle_to_turn > -179:
        server.send_key_input("mright " + str(turn_in_seconds) + " ")
    elif abs(angle_to_turn) < angle_threshold:
        turn_in_seconds = distance_to_target * 0.025
        if slow:
            turn_in_seconds /= 4
        elif power_through:
            turn_in_seconds *= 2
        if turn_in_seconds < 0.75 and not slow:
            turn_in_seconds = 0.75
        
        server.send_key_input("forward " + str(turn_in_seconds) + " ")
    sleep(0.2 + turn_in_seconds)





def deposit_balls(cv, boundrypixel, server):

    robotcenter, robotfront, robotDir = getRobotPosition(cv, boundrypixel)
    while robotcenter == None or robotDir == None:
        robotcenter, robotfront, robotDir = getRobotPosition(cv, boundrypixel)

    balls_deposited = False
    is_parked = False
    left_goal, right_goal = cv.get_goals()
    left_goal_normalized = convert_to_normalized(left_goal, boundrypixel["bottom_left"], boundrypixel["top_right"])
    right_goal_normalized = convert_to_normalized(right_goal, boundrypixel["bottom_left"], boundrypixel["top_right"])
    left_goal_distance = abs(euclidean_distance(robotcenter, left_goal))
    right_goal_distance = abs(euclidean_distance(robotcenter, right_goal))



    ##How far away from the goal should the robot be when depositing?
    deposit_distance = 25
    
    parked_threshold = 10

    target_goal = left_goal_normalized 
    target_park_position = (target_goal[0] + deposit_distance, target_goal[1])

    navigation_waypoints = []

    angle_threshold = 1.5

    while not balls_deposited:
        robotcenter, robotfront, robotDir = getRobotPosition(cv, boundrypixel)
        if robotDir == None or robotcenter == None:
            continue
        
        if abs(euclidean_distance(robotcenter, target_park_position)) < parked_threshold:
            is_parked = True

        
        if len(navigation_waypoints) == 0 and not is_parked:
            navigation_waypoints, target, is_ball = create_navigation_route(cv, boundrypixel, robotfront, target_park_position, convert_to_pixel(target_park_position, boundrypixel["bottom_left"], boundrypixel["top_right"]))
            target_position = target
            target_is_ball = False
            current_target_tries = 0
            if navigation_waypoints == None:
                navigation_waypoints = []

            cv.update_grid()
            continue
        
        
        
        target = navigation_waypoints[0] if not is_parked else target_goal
        vector_to_target = (target[0] - robotcenter[0], target[1] - robotcenter[1])
        
        angle_to_turn = angle_between((robotDir), vector_to_target)
        turn_in_seconds = abs((angle_to_turn/90)*0.85)
        
        if turn_in_seconds < 0.1:
            turn_in_seconds = 0.1
        
        if angle_threshold < angle_to_turn < 180:
            server.send_key_input("mleft " + str(turn_in_seconds)+ " ")
        elif -angle_threshold > angle_to_turn > -179:
            server.send_key_input("mright " + str(turn_in_seconds)+ " ")
        elif abs(angle_to_turn)<angle_threshold:
            if not is_parked:
                server.send_key_input("forward 0.75")
            elif not balls_deposited:
                server.send_key_input("k")
                balls_deposited = True
                sleep(5)
        sleep(0.2 + turn_in_seconds)

        robotmid = find_center_between_points(robotcenter, robotfront)

        distance_to_target = euclidean_distance(target, robotfront)
        origin_distance_to_target = euclidean_distance(target, robotcenter)
        mid_distance_to_target = euclidean_distance(target, robotmid)

        robot_points = [distance_to_target, mid_distance_to_target,  origin_distance_to_target] #if len(navigation_waypoints) > 1 else [origin_distance_to_target, mid_distance_to_target]
        if min(robot_points) < parked_threshold:
            if len(navigation_waypoints) > 0:
                del navigation_waypoints[0]
                continue
            break
            
        

def acquireTargetBall(cv, boundrypixel):
    ballspixelcords = cv.get_ball_positions()
    balls_normalized = []
    for coords in ballspixelcords:
        if coords is None:
            return None, None
        normalized_coords = convert_to_normalized(coords, boundrypixel["bottom_left"], boundrypixel["top_right"])
        balls_normalized.append(normalized_coords)

    
    robotcenter, robotfront, robotDir = getRobotPosition(cv, boundrypixel)
    if robotfront == None: return None, None
    print("Robot direction and angle: " + str(robotDir))

    # Calculate distances
    distances = [euclidean_distance(robotfront, coord)
                 for coord in balls_normalized]

    # Find the index of the coordinate with the smallest distance

    min_distance = 10

    closest_distance = None
    closest_index = None
    closest_coordinate = None
    closest_pixels = None

    # Filter distances greater than min_distance and find the minimum of the filtered list
    filtered_distances = [d for d in distances if d > min_distance]
    if filtered_distances:
        closest_distance = min(filtered_distances)
        closest_index = distances.index(closest_distance)
        closest_coordinate = balls_normalized[closest_index]
        closest_pixels = convert_to_pixel(closest_coordinate, boundrypixel["bottom_left"], boundrypixel["top_right"])
    
    return closest_coordinate, closest_pixels

