import cv2
import numpy as np
from collections import deque
from imutils.video import VideoStream
import threading
import sys
import math
import socket
from OpenCV.CVGrid import CVGrid
class CVInterface:

    test_mode = False
    
    ## How much the image is blurred (to make detection easier)
    __gaussian_blur = (27,27)

    ## Variables for for drawing frames when any of the methods are called.
    robot = None
    projection = None
    ball_pos = []
    confirmed_balls = []
    target_pos = None
    top_left = (0,0)
    top_right = (0,0)
    bottom_left = (0,0)
    bottom_right = (0,0)
    left_goal = (0,0)
    right_goal = (0,0)

    egg_keypoints = []

    waypoints = []

    __deposit_distance = 150

## BOUNDARY DETECTION (for detecting edges)
    ##All BOUND_X variables are parameters for the HoughLines detector. 
    ## See openCV documentation for what they mean
    __BOUND_DP = 10
    __BOUND_ANGLE_RES = np.pi/180
    __BOUND_THRESH = 30
    __BOUND_MINLEN = 50
    __BOUND_MAXGAP = 5
    __REFRESH_BOUND_FRAME = 25  ## How many frames should the program analyze when finding the boundary?
    __boundary_lower_color1 = np.array([0, 10, 125], dtype='uint8') ## Lower bound for color of the edge (HSV)
    __boundary_upper_color1 = np.array([6, 255, 255], dtype='uint8') ## Upper bound for color of the edge (HSV)
    __boundary_lower_color2 = np.array([174, 10, 125], dtype='uint8') ## Lower bound for color of the edge (HSV)
    __boundary_upper_color2 =  np.array([180, 255, 255], dtype='uint8') ## Upper bound for color of the edge (HSV)

    
    ## PATH FINDING
    GRID = None
    """
    A class defining methods for computer vision functionalities.
    """
    def __init__(self, video_capture_device : int):
        self.video_capture_device = video_capture_device
        self.__init_ball_detector()
        self.__init_robot_detector()
        self.__init_egg_detector()
        self.__capture_device = cv2.VideoCapture(video_capture_device)   ## Target capture device
        '''
        self.__capture_device = cv2.VideoCapture(video_capture_device, cv2.CAP_DSHOW)   ## Target capture device
        self.__capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 640*2)
        self.__capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 480*2)
        '''
    '''
    def __init__(self, test_picture : str):
        self.test_mode = True
        self.test_picture = test_picture
        self.__init_ball_detector()
        self.__init_robot_detector()
    '''     

    def __init_ball_detector(self):
        ## SETTINGS FOR BALL DETECTIONS
        params = cv2.SimpleBlobDetector_Params()
        ## See this link for explanation of threshold, area, circularity, convexity and intertia
        ## https://learnopencv.com/blob-detection-using-opencv-python-c/
        params.minThreshold = 0      ## Min threshold for when something is accepted as a ball
        params.maxThreshold = 200    ## Max threshold
        params.filterByArea = True   ## Should we use the area of the blobs to filter whether its a ball?
        params.minArea = 300       ## Potential setting for that balls needs to have an area greater than x
        params.maxArea = 500         ## If the blobs area is bigger than this, it will not be detected as a ball
        params.filterByCircularity = True  ## Should we use the circularity to filter?
        params.minCircularity = 0.4        ## Min circularity
        params.filterByConvexity = False    ## Should we use the convexity?
        params.minConvexity = 0.8
        params.filterByInertia = True     ## Should we use inertia?
        params.minInertiaRatio = 0.4
        self.__ball_detector = cv2.SimpleBlobDetector_create(params)

    def __init_egg_detector(self):
        ## SETTINGS FOR EGG DETECTIONS
        eggparams = cv2.SimpleBlobDetector_Params()
        ## See this link for explanation of threshold, area, circularity, convexity and intertia
        ## https://learnopencv.com/blob-detection-using-opencv-python-c/
        eggparams.minThreshold = 0      ## Min threshold for when something is accepted as a ball
        eggparams.maxThreshold = 200      ## Min threshold for when something is accepted as a ball
        eggparams.filterByArea = True   ## Should we use the area of the blobs to filter whether its a ball?
        eggparams.minArea = 2500
        eggparams.maxArea = 3500
        eggparams.filterByCircularity = True  ## Should we use the circularity to filter?
        eggparams.minCircularity = 0.25        ## Min circularity
        eggparams.filterByConvexity = True    ## Should we use the convexity?
        eggparams.minConvexity = 0.87
        eggparams.filterByInertia = True     ## Should we use inertia?
        eggparams.minInertiaRatio = 0.55
        self.__egg_detector = cv2.SimpleBlobDetector_create(eggparams)

    def  __init_robot_detector(self):
        ## SETTINGS FOR ROBOT DETECTIONS
        rparams = cv2.SimpleBlobDetector_Params()
        ## See this link for explanation of threshold, area, circularity, convexity and intertia
        ## https://learnopencv.com/blob-detection-using-opencv-python-c/
        rparams.minThreshold = 0
        rparams.maxThreshold = 200
        rparams.filterByCircularity = True
        rparams.filterByArea = True
        rparams.minArea = 500
        rparams.minCircularity = 0.4
        rparams.filterByConvexity = True
        rparams.minConvexity = 0.87 
        rparams.filterByInertia = False
        rparams.maxInertiaRatio = 0.5
        ## Upper and lower colors for detection of robot  (ALL IN HSV)
        self.__robot_detector = cv2.SimpleBlobDetector_create(rparams)
        self.__robot_origin_lower_color = np.array([45, 10, 125], dtype='uint8')  #Lower color of center (green circle)
        self.__robot_origin_upper_color = np.array([90, 255, 255], dtype='uint8') #Upper color of center
        self.__robot_direction_lower_color = np.array([8, 150, 125], dtype='uint8') #Lower color of direction marker (orange)
        self.__robot_direction_upper_color = np.array([35, 255, 255], dtype='uint8') #Upper color of direction marker
    def __cap_frame(self):
        if not self.test_mode:
            _, frame = self.__capture_device.read()
        else:
            frame = cv2.imread(self.test_picture)
        
        return frame

    def __update_drawing(self, frame):
        self.__draw_grid(frame)
        self.__draw_balls(self.ball_pos, frame)
        self.__draw_balls(self.confirmed_balls, frame, True)
        self.__draw_course(frame)
        self.__draw_robot(frame)
        self.__draw_target(frame)
        self.__draw_waypoints(frame)
        self.__draw_egg(frame)
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
    
    def __draw_grid(self, frame):
        if self.GRID is None:
            return

        # Create an overlay image
        overlay = frame.copy()

        for grid_column in self.GRID.cells:
            for cell in grid_column:
                if cell.status == "blocked":
                    top_left = (cell.pixel_position[0], cell.pixel_position[1])
                    bottom_right = (cell.pixel_position[0] + self.GRID.cell_size[0], cell.pixel_position[1] + self.GRID.cell_size[1])

                    # Draw filled rectangle on the overlay
                    color = (0, 255, 0) if cell.status == "unblocked" else (255, 0, 255)
                    cv2.rectangle(overlay, top_left, bottom_right, color, -1)

        # Transparency factor (0.0 - 1.0)
        alpha = 0.1

        # Add the overlay with transparency to the original frame
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


        # Draw the contour lines on the original frame
        for grid_column in self.GRID.cells:
            for cell in grid_column:
                if cell.status == "blocked":
                    top_left = (cell.pixel_position[0], cell.pixel_position[1])
                    bottom_right = (cell.pixel_position[0] + self.GRID.cell_size[0], cell.pixel_position[1] + self.GRID.cell_size[1])
                    color = (0, 255, 0) if cell.status == "unblocked" else (255, 0, 255)
                    cv2.rectangle(frame, top_left, bottom_right, color, 2)

    
    def update_grid(self):
        frame = self.__cap_frame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        self.GRID.clearBlocks()
        self.__block_egg_positions()
        self.block_cross_and_boundary(hsv)
        self.GRID.expand_block(3)
        self.GRID.block_out_of_bounds({
            "top_left": self.top_left,
            "top_right": self.top_right,
            "bottom_right": self.bottom_right,
            "bottom_left": self.bottom_left,
        })
        
        self.__draw_grid(frame)

    def get_grid(self):
        return self.GRID
    def block_cross_and_boundary(self, hsv):
        mask1 = cv2.inRange(hsv, self.__boundary_lower_color1, self.__boundary_upper_color1)
        mask2 = cv2.inRange(hsv, self.__boundary_lower_color2, self.__boundary_upper_color2)

        combined_mask = cv2.bitwise_or(mask1, mask2)

        cell_width, cell_height = self.GRID.cell_size
        threshold = 0.25  # 50% threshold

        for cell_y in range(0, combined_mask.shape[0], cell_height):
            for cell_x in range(0, combined_mask.shape[1], cell_width):
                cell = combined_mask[cell_y:cell_y + cell_height, cell_x:cell_x + cell_width]
                non_zero_count = np.count_nonzero(cell)
                total_pixels = cell.size

                if non_zero_count / total_pixels >= threshold:
                    center_x = cell_x + cell_width // 2
                    center_y = cell_y + cell_height // 2
                    self.GRID.block_at_pixel_position((center_x, center_y))


    def __find_robot_origin(self, frame):
        lower = self.__robot_origin_lower_color
        upper = self.__robot_origin_upper_color
        hsvIm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvIm, lower, upper)
        img = cv2.bitwise_and(hsvIm, hsvIm, mask = mask)
        orimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(orimg, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.__gaussian_blur, cv2.BORDER_DEFAULT)
        negative = cv2.bitwise_not(blur)
        keypoints = self.__robot_detector.detect(negative)
        return keypoints

    def __find_robot_direction(self, frame):
        lower = self.__robot_direction_lower_color
        upper = self.__robot_direction_upper_color
        hsvIm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsvIm, lower, upper)
        img = cv2.bitwise_and(hsvIm, hsvIm, mask = mask)
        orimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(orimg, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.__gaussian_blur, cv2.BORDER_DEFAULT)
        negative = cv2.bitwise_not(blur)
        
        keypoints = self.__robot_detector.detect(negative)
        return keypoints

    def __find_egg(self):
        frame = self.__cap_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.__gaussian_blur, cv2.BORDER_DEFAULT)
        negative = cv2.bitwise_not(blur)
        keypoints = self.__egg_detector.detect(negative)
        self.egg_keypoints = keypoints

    def __block_egg_positions(self):
        self.__find_egg()        
        for kp in self.egg_keypoints:
            x_center, y_center = int(kp.pt[0]), int(kp.pt[1])
            radius = int(kp.size // 2)
            
            # Calculate the bounding box of the region around the keypoint
            x_start = max(0, x_center - radius)
            x_end = min(self.GRID.frame_size[0], x_center + radius)
            y_start = max(0, y_center - radius)
            y_end = min(self.GRID.frame_size[1], y_center + radius)
            
            # Iterate through the bounding box and call block_at_pixel_position for each pixel
            for y in range(y_start, y_end):
                for x in range(x_start, x_end):
                    if (x - x_center) ** 2 + (y - y_center) ** 2 <= radius ** 2:
                        self.GRID.block_at_pixel_position((x, y))


    def __find_edges(self, frame):
        lower = self.__boundary_lower_color1
        upper = self.__boundary_upper_color1
        lower2 = self.__boundary_lower_color2
        upper2 = self.__boundary_upper_color2
        hsvIm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsvIm, lower, upper)
        mask2 = cv2.inRange(hsvIm, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        img = cv2.bitwise_and(hsvIm, hsvIm, mask = mask)
        orimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(orimg, cv2.COLOR_BGR2GRAY)

        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(
                edges,
                self.__BOUND_DP,
                self.__BOUND_ANGLE_RES,
                threshold=self.__BOUND_THRESH,
                minLineLength=self.__BOUND_MINLEN,
                maxLineGap=self.__BOUND_MAXGAP
                )
        return lines


    def __draw_waypoints(self, frame):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 1
        fontColor              = (255,0,0)
        thickness              = 3
        lineType               = 2
        if self.waypoints is not None:
            for point in self.waypoints:
                # draw the outer circle
                x = np.uint16(point[0])
                y = np.uint16(point[1])
                
                # draw the center of the circle
                cv2.circle(frame,(x,y ),2,(160,69,236),3)


    def __draw_balls(self, keypoints, frame, confirmed_highlight = False):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 1
        fontColor              = (255,0,0)
        thickness              = 3
        lineType               = 2
        if keypoints is not None:
            for point in keypoints:
                # draw the outer circle
                x = np.uint16(point.pt[0])
                y = np.uint16(point.pt[1])
                radius = int(np.uint16(point.size)/2)
                
                cv2.circle(frame, (x,y),radius,(0,255,0) if not confirmed_highlight else (255, 0, 0),2)
                # draw the center of the circle
                cv2.circle(frame,(x,y ),2,(0,0,255),3)
                cv2.putText(frame,'Ball',
                    (x,y),
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)
                
    def __draw_target(self,frame):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 1
        fontColor              = (0,0,255)
        thickness              = 3
        lineType               = 2
        if self.target_pos is not None:
            x = np.uint16(self.target_pos[0])
            y = np.uint16(self.target_pos[1])
            radius = int(15)
            
            
            cv2.circle(frame, (x,y),radius,(255,0,0),2)
            # draw the center of the circle
            cv2.circle(frame,(x,y ),2,(0,0,255),3)
            cv2.putText(frame,'Target',
                (x,y),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)
    def __draw_egg(self, frame):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 1
        fontColor              = (255,255,0)
        thickness              = 3
        lineType               = 2


        # Draw keypoints on the frame
        for kp in self.egg_keypoints:
            x, y = int(kp.pt[0]), int(kp.pt[1])
            radius = int(kp.size / 2)
            cv2.circle(frame, (x, y), radius, (0, 255, 255), thickness, lineType)
            cv2.putText(frame, 'Egg', (x, y), font, fontScale, fontColor, thickness, lineType)

    def __draw_course(self, frame):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,500)
        fontScale              = 1
        fontColor              = (255,0,0)
        thickness              = 3
        lineType               = 2
        cv2.putText(frame,'Bottom left: ' + str(self.bottom_left),
            (self.bottom_left[0],self.bottom_left[1]),
            font,
            fontScale,
            fontColor,
            thickness,
            lineType)


        cv2.putText(frame,'Bottom right: ' + str(self.bottom_right),
            (self.bottom_right[0], self.bottom_right[1]),
            font,
            fontScale,
            fontColor,
            thickness,
            lineType)


        cv2.putText(frame,'Top right: ' + str(self.top_right),
            (self.top_right[0], self.top_right[1]),
            font,
            fontScale,
            fontColor,
            thickness,
            lineType)


        cv2.putText(frame,'top_left: ' + str(self.top_left),
            (self.top_left[0], self.top_left[1]),
            font,
            fontScale,
            fontColor,
            thickness,
            lineType)
        cv2.line(frame,(self.top_left[0], self.top_left[1]),(self.top_right[0], self.top_right[1]),(0,255,0),5)
        cv2.line(frame,(self.bottom_left[0], self.bottom_left[1]),(self.bottom_right[0], self.bottom_right[1]),(0,255,0),5)
        cv2.line(frame,(self.bottom_left[0], self.bottom_left[1]),(self.top_left[0], self.top_left[1]),(0,255,0),5)
        cv2.line(frame,(self.top_right[0], self.top_right[1]),(self.bottom_right[0], self.bottom_right[1]),(0,255,0),5)
        cv2.circle(frame, self.left_goal,15,(255,255,0),2)
        cv2.circle(frame, (self.left_goal[0] + self.__deposit_distance, self.left_goal[1]),15,(255,255,0),2)
        # draw the center of the circle
        cv2.putText(frame,'Left goal',
            self.left_goal,
            font,
            fontScale,
            fontColor,
            thickness,
            lineType)
        cv2.circle(frame, self.right_goal,15,(255,255,0),2)
        cv2.circle(frame, (self.right_goal[0] - self.__deposit_distance, self.right_goal[1]),15,(255,255,0),2)
        # draw the center of the circle
        cv2.putText(frame,'Right goal',
            self.right_goal,
            font,
            fontScale,
            fontColor,
            thickness,
            lineType)
    def __draw_robot(self, frame):

        if(self.robot is None):
            return
        
        origin = self.robot["origin"]
        dir_vector = self.robot["direction"]

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10,500)
        fontScale              = 1
        fontColor              = (255,0,0)
        thickness              = 3
        lineType               = 2

        if origin is not None:
            p = origin 
            x = np.uint16(p.pt[0])
            y = np.uint16(p.pt[1])
            radius = int(np.uint16(p.size)/2)
            cv2.circle(frame, (x,y),radius,(0,255,0),2)
            cv2.circle(frame,(x,y ),2,(0,0,255),3)
            cv2.putText(frame,'Robot center',
                (x,y),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)
        if dir_vector is not None:
            p = dir_vector
            x = np.uint16(p.pt[0])
            y = np.uint16(p.pt[1])
            radius = int(np.uint16(p.size)/2)
            cv2.circle(frame, (x,y),radius,(0,255,0),2)
            cv2.circle(frame,(x,y ),2,(0,0,255),3)
            cv2.putText(frame,'Robot direction',
                (x,y),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)
        if self.projection is not None:
            cv2.circle(frame, (int(self.projection["r_front"][0]), int(self.projection["r_front"][1])),25,(0,255,255),2)
            cv2.circle(frame, (int(self.projection["r_front"][0]), int(self.projection["r_front"][1])),25,(0,255,255),2)
            cv2.circle(frame, (int(self.projection["r_center"][0]), int(self.projection["r_center"][1])),25,(0,255,255),2)
            cv2.circle(frame, (int(self.projection["r_center"][0]), int(self.projection["r_center"][1])),25,(0,255,255),2)

    def get_robot_position_and_rotation(self):
        frame = self.__cap_frame()
        robot_origin_candidates = self.__find_robot_origin(frame)
        robot_direction_candidates = self.__find_robot_direction(frame)
        origin = robot_origin_candidates[0] if len(robot_origin_candidates) > 0 else None
        direction = robot_direction_candidates[0] if len(robot_direction_candidates) > 0 else None
        data = {"origin": origin.pt if origin is not None else None, "direction": direction.pt if direction is not None else None}
        self.robot = {"origin": origin, "direction": direction}
        self.__update_drawing(frame)
        return data
    
    def get_ball_positions_across_frames(self, frame_count):
        confirmed_balls = []
        ball_equality_threshold = 10
        for i in range(frame_count):
            new_detections = self.get_ball_positions()
            if i == 0:
                confirmed_balls = new_detections
                continue
            new_confirmations = []
            for ball in confirmed_balls:
                for new_ball in new_detections:
                    if abs(new_ball[0] - ball[0]) < ball_equality_threshold and abs(new_ball[1] - ball[1]) < ball_equality_threshold:
                        new_confirmations.append(new_ball)
            confirmed_balls = new_confirmations
        return confirmed_balls
                        




    def get_ball_positions(self):
        frame = self.__cap_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.__gaussian_blur, cv2.BORDER_DEFAULT)
        negative = cv2.bitwise_not(blur)
        keypoints = self.__ball_detector.detect(negative)
        for keypoint in keypoints:
            if not self.top_left[0] < np.uint16(keypoint.pt[0]) < self.top_right[0]:
                keypoints.remove(keypoint)
                continue
            if not self.top_left[1] < np.uint16(keypoint.pt[1]) < self.bottom_left[1]:
                keypoints.remove(keypoint)
        ball_positions = [(np.uint16(point.pt[0]), np.uint16(point.pt[1])) for point in keypoints]
        self.ball_pos = keypoints

        self.__update_drawing(frame)
        return ball_positions


    def get_course_boundary(self):
        frame = self.__cap_frame()
        vertical_center = int(frame.shape[0]/2)
        horizontal_center = int(frame.shape[1]/2)
        self.top_left = [horizontal_center, vertical_center]
        self.top_right  = [horizontal_center, vertical_center]
        self.bottom_left  = [horizontal_center, vertical_center]
        self.bottom_right = [horizontal_center, vertical_center]
    
        for _ in range(self.__REFRESH_BOUND_FRAME):
            frame = self.__cap_frame()
            lines = self.__find_edges(frame)
            if lines is None:
                continue
                ##Idea is to get the corners of the lines that are closes to the edge of the frame
            for points in lines:
                x1,y1,x2,y2=points[0]
                delta_y = y2-y1
                delta_x = x2-x1
                slope = math.atan2(delta_y, delta_x)
                slope_type = "Unknown"

                ## Use the slope of the lines to determine if its vertical or horizontal
                ## Change the y-coordinates of the course corners if they are further towards
                ## the edge of the frame than the current lines.

                ## Is line vertical?
                if 1.3 < abs(slope) < 1.6:
                    start_x, start_y = x1, y1

                    ## Make sure that the start of the line is the point with smallest y
                    end_x, end_y = x2, y2
                    if y2 < y1:
                        start_x, start_y = x2, y2
                        end_x, end_y = x1, y1

                    ## If the start of the line is further to the left than the horizontal center,
                    ## the line is a 'Left' line.
                    if start_x <  horizontal_center:
                        slope_type = "Left"

                        ## If the y position of the start of the line is above the middle,
                        ## update the perceived top left corner of the course. Change the
                        ## bottom left if otherwise, as the line is below the middle
                        if start_y < self.top_left[1]:
                            self.top_left[1] = start_y   # top_left y
                        if end_y > self.bottom_left[1]:
                            self.bottom_left[1] = end_y  # bottem_left y

                            ## Do the same with the lines that are to the right of the horizontal_center, but
                            ## update top right and bottom right instead
                    else:
                        slope_type = "Right"
                        if start_y < self.top_right[1]:
                            self.top_right[1] = start_y  # top right y
                        if end_y > self.bottom_right[1]:
                            self.bottom_right[1] = end_y # bottom right y

                ## Is line horizontal?
                ## Much like the previous section, the line is registered as being horizontal if the slope is
                ## between a certain interval. Then it is checked wether it is in the top or bottom depending on its
                ## y coordinates. Afterwards it is checked wether wether is it right or left and the corners' x values updated
                if -0.5 < slope < 0.5:
                    start_x, start_y = x1, y1
                    end_x, end_y = x2, y2
                    ## Make sure that the start of the line is the point with smallest x
                    if x2 < x1:
                        start_x, start_y = x2, y2
                        end_x, end_y = x1, y1

                    if start_y <  vertical_center:
                        slope_type = "Top"
                        if start_x < self.top_left[0]:
                            self.top_left[0] = start_x # top left x
                        if end_x > self.top_right[0]:
                            self.top_right[0] = end_x # top right x
                    else:
                        slope_type = "Bottom"
                        if start_x < self.bottom_left[0]:
                            self.bottom_left[0] = start_x # bottom left x
                        if end_x > self.bottom_right[0]:
                            self.bottom_right[0] = end_x # bottom right x
        self.__update_drawing(frame)
        return {
            "top_left": self.top_left,
            "top_right": self.top_right,
            "bottom_right": self.bottom_right,
            "bottom_left": self.bottom_left,
        }
                


    def get_cross_position(self):
        """
        This method should return the starting line cross position.
        (Implementation details will depend on the specific vision system)
        """
       



    def get_egg_position(self):
        """
        This method should return the egg position data.
        (Implementation details will depend on the specific vision system)
        """
        pass


    def get_goals(self):
        global top_left, bottom_left, top_right, bottom_right
        left_goal_center = (int((self.top_left[0]+self.bottom_left[0])/2), int(self.top_left[1]-((self.top_left[1]-self.bottom_left[1])/2)))
        right_goal_center = (int((self.top_right[0]+self.bottom_right[0])/2), int(self.top_right[1]-((self.top_right[1]-self.bottom_right[1])/2)))
        self.left_goal = left_goal_center
        self.right_goal = right_goal_center
        return (left_goal_center, right_goal_center)


    def get_center(self):
        frame = self.__cap_frame()
        return (int(frame.shape[1]/2), int(frame.shape[0]/2))
    

    def initialize_grid(self, grid_size):
        frame = self.__cap_frame()
        self.GRID = CVGrid(grid_size, (1920,1080))
        self.update_grid()


''' Example usage
inter = CVInterface(0)
print(inter.get_robot_position_and_rotation())
print(inter.get_ball_positions())
print(inter.get_course_boundary()[])
'''