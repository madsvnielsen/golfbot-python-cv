import math
from time import sleep
from CVInterface import CVInterface
from Server1 import Server
def start():
    cv =CVInterface(0)
    server = Server()
    boundrypixel= cv.get_course_boundary()

    while True:
        

        ballspixelcords= cv.get_ball_positions()

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

        balls_normalized = []
        
        for coords in ballspixelcords:
            if coords is None:
                continue
            
            normalized_coords = convert_to_normalized(coords, boundrypixel["bottom_left"], boundrypixel["top_right"])
            balls_normalized.append(normalized_coords)
        robotpixel=cv.get_robot_position_and_rotation()
        robotcenter= convert_to_normalized(robotpixel["origin"],boundrypixel["bottom_left"], boundrypixel["top_right"])
        robotfront= convert_to_normalized(robotpixel["direction"],boundrypixel["bottom_left"], boundrypixel["top_right"])
        
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
        robotDir= find_robot_direction_and_angle(robotcenter,robotfront)
        print(robotDir)
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

        # Calculate distances
        distances = [euclidean_distance(robotcenter, coord) for coord in balls_normalized]

        # Find the index of the coordinate with the smallest distance
        closest_index = distances.index(min(distances))

        # Get the closest coordinate
        closest_coordinate = balls_normalized[closest_index]
        vector_to_closest_coordinate = (closest_coordinate[0] - robotcenter[0], closest_coordinate[1] - robotcenter[1])
        print(closest_coordinate)
        print(vector_to_closest_coordinate)

        #calculate angle to turn
        angle_to_turn = angle_between((robotDir), vector_to_closest_coordinate)
        print(angle_to_turn)
        
        if 5 < angle_to_turn < 180:
            server.send_key_input("left")
        elif -5 > angle_to_turn > -179:
            server.send_key_input("right")
        elif abs(angle_to_turn)<5:
            server.send_key_input("forward")
        sleep(0.5)
        '''
        for keypoints in start_generator:
            balls = keypoints
            print('cordinates'+balls)

        robot=cv2.KeyPoint(100, 200, _size=10, _angle=0, _response=0, _octave=0, _class_id=-1)
        '''

start()
