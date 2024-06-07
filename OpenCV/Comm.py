import math
from time import sleep
from CVInterface import CVInterface
from Server1 import Server



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

def find_robot_direction_and_angle(center_coords, front_coords):
        # Calculate the vector representing the direction the robot is facing
        direction = (front_coords[0] - center_coords[0], front_coords[1] - center_coords[1])
        '''
        # Calculate the angle between the direction vector and the x-axis
        angle = math.degrees(math.atan2(direction[1], direction[0]))
        if angle < 0:
            angle += 360  # Ensure the angle is positive
        '''
        return direction


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
    robotpixel=cv.get_robot_position_and_rotation()
    
    print("Robot pixel", robotpixel)
    if(robotpixel["origin"] == None or robotpixel["direction"] == None):
        return None, None, None
    robotcenter= convert_to_normalized(robotpixel["origin"],boundrypixel["bottom_left"], boundrypixel["top_right"])
    robotfront= convert_to_normalized(robotpixel["direction"],boundrypixel["bottom_left"], boundrypixel["top_right"])
    
    return robotcenter, robotfront, find_robot_direction_and_angle(robotcenter,robotfront)

def start():
    cv =CVInterface(1)
    server = Server()
    boundrypixel= cv.get_course_boundary()

    active_target = None
    search_mode = True
    
    assumed_balls_in_mouth = 0
    pickup_threshold = 5

    while True:
        if search_mode:
             active_target = acquireTargetBall(cv, boundrypixel)
             if active_target != None:
                search_mode = False
        
        robotcenter, robotfront, robotDir = getRobotPosition(cv, boundrypixel)
        if robotcenter == None: continue
        vector_to_active_target = (active_target[0] - robotcenter[0], active_target[1] - robotcenter[1])
        print("TARGET: " + str(active_target))
        print("VECTOR: " + str(vector_to_active_target))
        distance_to_target = euclidean_distance(active_target, robotfront)
        print("DISTANCE: " + str(distance_to_target))
        
        if distance_to_target < pickup_threshold:
            assumed_balls_in_mouth += 1
            search_mode = True
            continue
            



        #calculate angle to turn
        angle_to_turn = angle_between((robotDir), vector_to_active_target)
        print(angle_to_turn)
        turn_in_seconds = abs((angle_to_turn/90)*0.85)
        
        if 3 < angle_to_turn < 180:
            server.send_key_input("left " + str(turn_in_seconds)+ "\n")
        elif -3 > angle_to_turn > -179:
            server.send_key_input("right " + str(turn_in_seconds)+ "\n")
        elif abs(angle_to_turn)<3:
            server.send_key_input("forward")
        sleep(0.5)
        '''
        for keypoints in start_generator:
            balls = keypoints
            print('cordinates'+balls)

        robot=cv2.KeyPoint(100, 200, _size=10, _angle=0, _response=0, _octave=0, _class_id=-1)
        '''

def acquireTargetBall(cv, boundrypixel):
    ballspixelcords= cv.get_ball_positions()
    balls_normalized = []
    for coords in ballspixelcords:
        if coords is None:
            return None
        normalized_coords = convert_to_normalized(coords, boundrypixel["bottom_left"], boundrypixel["top_right"])
        balls_normalized.append(normalized_coords)

    
    robotcenter,_, robotDir = getRobotPosition(cv, boundrypixel)
    if robotcenter == None: return None
    print("Robot direction and angle: " + str(robotDir))

            # Calculate distances
    distances = [euclidean_distance(robotcenter, coord) for coord in balls_normalized]

            # Find the index of the coordinate with the smallest distance
    closest_index = distances.index(min(distances))

            # Get the closest coordinate
    closest_coordinate = balls_normalized[closest_index]
    return closest_coordinate

start()
