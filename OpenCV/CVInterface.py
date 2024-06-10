import cv2
import numpy as np
from collections import deque
from imutils.video import VideoStream
import threading
import sys
import math
import socket

class CVInterface:
    
    ## How much the image is blurred (to make detection easier)
    __gaussian_blur = (27,27)

    ## Variables for for drawing frames when any of the methods are called.
    robot = None
    ball_pos = []
    target_pos = None
    top_left = (0,0)
    top_right = (0,0)
    bottom_left = (0,0)
    bottom_right = (0,0)

## BOUNDARY DETECTION (for detecting edges)
    ##All BOUND_X variables are parameters for the HoughLines detector. 
    ## See openCV documentation for what they mean
    __BOUND_DP = 10
    __BOUND_ANGLE_RES = np.pi/180
    __BOUND_THRESH = 30
    __BOUND_MINLEN = 50
    __BOUND_MAXGAP = 5
    __REFRESH_BOUND_FRAME = 100  ## How many frames should the program analyze when finding the boundary?
    __boundary_lower_color = np.array([0, 0, 200], dtype='uint8') ## Lower bound for color of the edge (BGR)
    __boundary_upper_color = np.array([90, 90, 255], dtype='uint8') ## Upper bound for color of the edge (BGR)
  
    """
    A class defining methods for computer vision functionalities.
    """
    def __init__(self, video_capture_device):
        self.video_capture_device = video_capture_device
        self.__init_ball_detector()
        self.__init_robot_detector()
        self.__capture_device = cv2.VideoCapture(1)   ## Target capture device

    def __init_ball_detector(self):
        ## SETTINGS FOR BALL DETECTIONS
        params = cv2.SimpleBlobDetector_Params()
        ## See this link for explanation of threshold, area, circularity, convexity and intertia
        ## https://learnopencv.com/blob-detection-using-opencv-python-c/
        params.minThreshold = 0      ## Min threshold for when something is accepted as a ball
        params.maxThreshold = 200    ## Max threshold
        params.filterByArea = True   ## Should we use the area of the blobs to filter whether its a ball?
        ##params.minArea = 110       ## Potential setting for that balls needs to have an area greater than x
        params.maxArea = 500         ## If the blobs area is bigger than this, it will not be detected as a ball
        params.filterByCircularity = True  ## Should we use the circularity to filter?
        params.minCircularity = 0.4        ## Min circularity
        params.filterByConvexity = True    ## Should we use the convexity?
        params.minConvexity = 0.87        
        params.filterByInertia = False     ## Should we use inertia?
        params.maxInertiaRatio = 0.5
        self.__ball_detector = cv2.SimpleBlobDetector_create(params)

    def  __init_robot_detector(self):
        ## SETTINGS FOR ROBOT DETECTIONS
        rparams = cv2.SimpleBlobDetector_Params()
        ## See this link for explanation of threshold, area, circularity, convexity and intertia
        ## https://learnopencv.com/blob-detection-using-opencv-python-c/
        rparams.minThreshold = 0
        rparams.maxThreshold = 200
        rparams.filterByCircularity = True
        rparams.filterByArea = True
        rparams.minArea = 110
        rparams.minCircularity = 0.4
        rparams.filterByConvexity = True
        rparams.minConvexity = 0.87 
        rparams.filterByInertia = False
        rparams.maxInertiaRatio = 0.5
        ## Upper and lower colors for detection of robot  (ALL IN HSV)
        self.__robot_detector = cv2.SimpleBlobDetector_create(rparams)
        self.__robot_origin_lower_color = np.array([40, 10, 125], dtype='uint8')  #Lower color of center (green cirlcle)
        self.__robot_origin_upper_color = np.array([120, 255, 255], dtype='uint8') #Upper color of center
        self.__robot_direction_lower_color = np.array([137, 10, 125], dtype='uint8') #Lower color of direction marker (purple)
        self.__robot_direction_upper_color = np.array([170, 255, 255], dtype='uint8') #Upper color of direction marker
    def __cap_frame(self):
        _, frame = self.__capture_device.read()

        self.__draw_balls(self.ball_pos, frame)
        self.__draw_course(frame)
        self.__draw_robot(frame)

        self.__draw_target(frame)
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
        return frame

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

    def __find_edges(self, frame):
        lower = self.__boundary_lower_color
        upper = self.__boundary_upper_color
        mask = cv2.inRange(frame, lower, upper)
        img = cv2.bitwise_and(frame, frame, mask = mask)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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


    def __draw_balls(self, keypoints, frame):
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
                
                
                cv2.circle(frame, (x,y),radius,(0,255,0),2)
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
        fontColor              = (255,0,0)
        thickness              = 3
        lineType               = 2
        if self.target_pos is not None:
        
            x = np.uint16(self.target_pos[0])
            y = np.uint16(self.target_pos[1])
            radius = int(5)
            
            
            cv2.circle(frame, (x,y),radius,(255,0,0),2)
            # draw the center of the circle
            cv2.circle(frame,(x,y ),2,(255,0,0),3)
            cv2.putText(frame,'Target',
                (x,y),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)
                
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

    def get_robot_position_and_rotation(self):
        frame = self.__cap_frame()
        robot_origin_candidates = self.__find_robot_origin(frame)
        robot_direction_candidates = self.__find_robot_direction(frame)
        origin = robot_origin_candidates[0] if len(robot_origin_candidates) > 0 else None
        direction = robot_direction_candidates[0] if len(robot_direction_candidates) > 0 else None
        data = {"origin": origin.pt if origin is not None else None, "direction": direction.pt if direction is not None else None}
        self.robot = {"origin": origin, "direction": direction}
        return data


    def get_ball_positions(self):
        frame = self.__cap_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.__gaussian_blur, cv2.BORDER_DEFAULT)
        negative = cv2.bitwise_not(blur)
        keypoints = self.__ball_detector.detect(negative)
        ball_positions = [(np.uint16(point.pt[0]), np.uint16(point.pt[1])) for point in keypoints]
        self.ball_pos = keypoints
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
        pass


    def get_egg_position(self):
        """
        This method should return the egg position data.
        (Implementation details will depend on the specific vision system)
        """
        pass


    def get_goals(self):
        """
        This method should return the goal positions.
        (Implementation details will depend on the specific vision system)
        """
        pass

''' Example usage
inter = CVInterface(0)
print(inter.get_robot_position_and_rotation())
print(inter.get_ball_positions())
print(inter.get_course_boundary()[])
'''