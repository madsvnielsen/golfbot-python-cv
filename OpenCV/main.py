import cv2
import numpy as np
from collections import deque
from imutils.video import VideoStream
import threading
import sys
import math
import socket

window_name = "Window Name"

## BALL DETECTION
GAUSSIAN_BLUR = (27,27)
EDGE_CANNY_THRESHOLD = 60
EDGE_ACCUMULATOR_THRESHOLD = 60
HOUGH_DP = 2
HOUGH_MIN_DISTANCE = 50
HOUGH_MIN_RADIUS = 5
HOUGH_MAX_RADIUS = 25


## BALL DETECTION V2
params = cv2.SimpleBlobDetector_Params()
params.minThreshold = 0
params.maxThreshold = 200
params.filterByArea = True
##params.minArea = 110
params.maxArea = 500
params.filterByCircularity = True
params.minCircularity = 0.4
params.filterByConvexity = True
params.minConvexity = 0.87 
params.filterByInertia = False
params.maxInertiaRatio = 0.5
detector = cv2.SimpleBlobDetector_create(params)


## ROBOT ORIGIN DETECTION
rparams = cv2.SimpleBlobDetector_Params()
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
rdetector = cv2.SimpleBlobDetector_create(rparams)

## EGG DETECTION
attrib = cv2.SimpleBlobDetector_Params()
attrib.minThreshold = 0
attrib.maxThreshold = 200
attrib.filterByArea = True
attrib.minArea = 2600
attrib.filterByCircularity = True
attrib.minCircularity = 0.25

attrib.filterByConvexity = True
attrib.minConvexity = 0.87
attrib.filterByInertia = True
attrib.minInertiaRatio = 0.55
detection = cv2.SimpleBlobDetector_create(attrib)

## BOUNDARY DETECTION
BOUND_DP = 10
BOUND_ANGLE_RES = np.pi/180
BOUND_THRESH = 30
BOUND_MINLEN = 50
BOUND_MAXGAP = 5
REFRESH_BOUND_FRAME = 20
CURRENT_FRAME = 0
FIND_COURSE = True


top_left = [0,0]
top_right = [0,0]
bottom_left = [0,0]
bottom_right = [0,0]


## Cross FINDING
template = cv2.imread("cross_template.png")
cross_startX, cross_startY, cross_endX, cross_endY = (0,0,0,0)


def monitor_cmd():
    global GAUSSIAN_BLUR, EDGE_ACCUMULATOR_THRESHOLD,  EDGE_CANNY_THRESHOLD,HOUGH_DP, HOUGH_MIN_RADIUS, HOUGH_MIN_DISTANCE, HOUGH_MAX_RADIUS,BOUND_DP, BOUND_ANGLE_RES, BOUND_THRESH, BOUND_MINLEN, BOUND_MAXGAP
    while True:
        print("""
        CURRENT SETTINGS
        **POST PROCESSING**
        BLUR = %s

        **CIRCLE EDGE DETECTION**
        CANNY = %s
        ACCUM = %s

        **CIRCLE DETECTION**
        DP = %s
        MINDIST = %s
        MINRAD = %s
        MAXRAD = %s

        **BOUNDARY DETECTION**
        BOUNDDP = %s
        ANGLERES = %s
        BOUNDTHRESH = %s
        MINLEN = %s
        MAXPGAP = %s

        Type SETTING <value> to change.

        """
        % (GAUSSIAN_BLUR[0],
         EDGE_CANNY_THRESHOLD,
         EDGE_ACCUMULATOR_THRESHOLD,
         HOUGH_DP, HOUGH_MIN_DISTANCE,
         HOUGH_MIN_RADIUS,
         HOUGH_MAX_RADIUS,
         BOUND_DP,
         BOUND_ANGLE_RES,
         BOUND_THRESH,
         BOUND_MINLEN,
         BOUND_MAXGAP
         )
         )
        i = input("> ").split(" ")

        c = i[0].strip()
        s = "NULL"
        if len(i) > 1:
            s = i[1].strip()

        if c == "BLUR":
            GAUSSIAN_BLUR = (int(s), int(s))

        if c == "CANNY":
            EDGE_CANNY_THRESHOLD = float(s)

        if c == "ACCUM":
            EDGE_ACCUMULATOR_THRESHOLD = float(s)
        if c == "DP":
            HOUGH_DP = float(s)
        if c == "MINDIST":
            HOUGH_MIN_DISTANCE = int(s)
        if c == "MINRAD":
            HOUGH_MIN_RADIUS = int(s)
        if c == "MAXRAD":
            HOUGH_MAX_RADIUS = int(s)
        if c == "BOUNDDP":
            BOUND_DP = float(s)
        if c == "ANGLERES":
            BOUND_ANGLE_RES = np.pi/float(s)
        if c == "BOUNDTHRESH":
            BOUND_THRESH = int(s)
        if c == "MINLEN":
            BOUND_MINLEN = int(s)
        if c == "MAXGAP":
            BOUND_MAXGAP = int(s)

'''
def balls(frame):

    lower = np.array([200, 200, 200], dtype='uint8')
    upper = np.array([255, 255, 255], dtype='uint8')

    mask = cv2.inRange(frame, lower, upper)
    img = cv2.bitwise_and(frame, frame, mask = mask)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hist = cv2.GaussianBlur(gray, GAUSSIAN_BLUR, cv2.BORDER_DEFAULT)
    
    blurred = hist#cv2.equalizeHist(hist)

    edges = cv2.Canny(blurred,EDGE_CANNY_THRESHOLD,EDGE_ACCUMULATOR_THRESHOLD)
    cimg = cv2.cvtColor(blurred,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,
    HOUGH_DP,HOUGH_MIN_DISTANCE,
    param1=EDGE_CANNY_THRESHOLD,
    param2=EDGE_ACCUMULATOR_THRESHOLD,
    minRadius=HOUGH_MIN_RADIUS,
    maxRadius=HOUGH_MAX_RADIUS)
    return (circles, edges)
'''

def balls2(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, GAUSSIAN_BLUR, cv2.BORDER_DEFAULT)
    negative = cv2.bitwise_not(blur)
    keypoints = detector.detect(negative)
    return keypoints

## Egg recognition
def bigEgg(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, GAUSSIAN_BLUR, cv2.BORDER_DEFAULT)
    negative = cv2.bitwise_not(blur)
    centerPoint = detection.detect(negative)
    return centerPoint


def find_edges(frame):

    lower = np.array([0, 0, 200], dtype='uint8')
    upper = np.array([90, 90, 255], dtype='uint8')

    mask = cv2.inRange(frame, lower, upper)
    img = cv2.bitwise_and(frame, frame, mask = mask)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(
            edges,
            BOUND_DP,
            BOUND_ANGLE_RES,
            threshold=BOUND_THRESH,
            minLineLength=BOUND_MINLEN,
            maxLineGap=BOUND_MAXGAP
            )
    return lines, edges, img

def find_robot_origin(frame):
    lower = np.array([40, 10, 125], dtype='uint8')
    upper = np.array([70, 255, 255], dtype='uint8')
    hsvIm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsvIm, lower, upper)
    img = cv2.bitwise_and(hsvIm, hsvIm, mask = mask)
    orimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(orimg, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, GAUSSIAN_BLUR, cv2.BORDER_DEFAULT)
    negative = cv2.bitwise_not(blur)
    keypoints = rdetector.detect(negative)
    return keypoints

def find_robot_direction(frame):
    lower = np.array([137, 10, 125], dtype='uint8')
    upper = np.array([170, 255, 255], dtype='uint8')
    hsvIm = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsvIm, lower, upper)
    img = cv2.bitwise_and(hsvIm, hsvIm, mask = mask)
    orimg = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(orimg, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, GAUSSIAN_BLUR, cv2.BORDER_DEFAULT)
    negative = cv2.bitwise_not(blur)
    keypoints = rdetector.detect(negative)
    return keypoints
    


def update_course_edges(lines, horizontal_center, vertical_center):
    if lines is not None:
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
                    if start_y < top_left[1]:
                        top_left[1] = start_y   # top_left y
                    if end_y > bottom_left[1]:
                        bottom_left[1] = end_y  # bottem_left y

                        ## Do the same with the lines that are to the right of the horizontal_center, but
                        ## update top right and bottom right instead
                else:
                    slope_type = "Right"
                    if start_y < top_right[1]:
                        top_right[1] = start_y  # top right y
                    if end_y > bottom_right[1]:
                        bottom_right[1] = end_y # bottom right y

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
                    if start_x < top_left[0]:
                        top_left[0] = start_x # top left x
                    if end_x > top_right[0]:
                        top_right[0] = end_x # top right x
                else:
                    slope_type = "Bottom"
                    if start_x < bottom_left[0]:
                        bottom_left[0] = start_x # bottom left x
                    if end_x > bottom_right[0]:
                        bottom_right[0] = end_x # bottom right x

## Finds the cross on the field via simple template matching
def find_cross(frame):
    global template, cross_startX, cross_startY, cross_endX, cross_endY
    imageGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    templateGray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    result = cv2.matchTemplate(imageGray, templateGray,
	cv2.TM_CCOEFF_NORMED)
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
    (cross_startX, cross_startY) = maxLoc
    cross_endX = cross_startX + template.shape[1]
    cross_endY = cross_startY + template.shape[0]

'''
## Draws balls (given by a result of HoughCircles) on a frame
def draw_balls(circles, frame):
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (255,0,0)
    thickness              = 3
    lineType               = 2
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            # draw the outer circle
            cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
            cv2.putText(frame,'Ball',
                (i[0],i[1]),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)
'''
def draw_balls2(keypoints, frame):
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
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


def draw_eggs(keypoints, frame):
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10, 500)
    fontScale = 1
    fontColor = (255, 0, 0)
    thickness = 3
    lineType = 2
    if keypoints is not None:

        for point in keypoints:
            # draw the outer circle
            x = np.uint16(point.pt[0])
            y = np.uint16(point.pt[1])
            radius = int(np.uint16(point.size) / 2)

            cv2.circle(frame, (x, y), radius, (255, 0, 0), 2)
            # draw the center of the circle
            cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
            cv2.putText(frame, 'egg',
                        (x, y),
                        font,
                        fontScale,
                        fontColor,
                        thickness,
                        lineType)

def draw_course(frame):
    global top_left, top_right, bottom_right, bottom_left
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (255,0,0)
    thickness              = 3
    lineType               = 2
    cv2.putText(frame,'Bottom left: ' + str(bottom_left),
        (bottom_left[0],bottom_left[1]),
        font,
        fontScale,
        fontColor,
        thickness,
        lineType)


    cv2.putText(frame,'Bottom right: ' + str(bottom_right),
        (bottom_right[0],bottom_right[1]),
        font,
        fontScale,
        fontColor,
        thickness,
        lineType)


    cv2.putText(frame,'Top right: ' + str(top_right),
        (top_right[0],top_right[1]),
        font,
        fontScale,
        fontColor,
        thickness,
        lineType)


    cv2.putText(frame,'top_left: ' + str(top_left),
        (top_left[0],top_left[1]),
        font,
        fontScale,
        fontColor,
        thickness,
        lineType)
    cv2.line(frame,(top_left[0], top_left[1]),(top_right[0], top_right[1]),(0,255,0),5)
    cv2.line(frame,(bottom_left[0], bottom_left[1]),(bottom_right[0], bottom_right[1]),(0,255,0),5)
    cv2.line(frame,(bottom_left[0], bottom_left[1]),(top_left[0], top_left[1]),(0,255,0),5)
    cv2.line(frame,(top_right[0], top_right[1]),(bottom_right[0], bottom_right[1]),(0,255,0),5)


def draw_cross(frame):
    global cross_startX, cross_startY, cross_endX, cross_startY
    cv2.rectangle(frame, (cross_startX, cross_startY), (cross_endX, cross_endY), (255, 0, 0), 3)



def QR_Reader(frame, ret):
    qcd = cv2.QRCodeDetector()


    if ret:
        ret_qr, decoded_info, points, _ = qcd.detectAndDecodeMulti(frame)
        if ret_qr:
            for s, p in zip(decoded_info, points):
                if s:
                    print(s)
                    color = (0, 255, 0)
                else:
                    color = (0, 0, 255)
                return cv2.polylines(frame, [p.astype(int)], True, color, 8)
      

def start():
    global FIND_COURSE, CURRENT_FRAME, REFRESH_BOUND_FRAME, top_left, top_right, bottom_right, bottom_left
    cap = cv2.VideoCapture(1)
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (255,0,0)
    thickness              = 3
    lineType               = 2
    
    ret, frame = cap.read()

    vertical_center = int(frame.shape[0]/2)
    horizontal_center = int(frame.shape[1]/2)
    ##Initialize the course corner-coordinates to be in the center of the frame.
    top_left = [horizontal_center, vertical_center]
    top_right  = [horizontal_center, vertical_center]
    bottom_left  = [horizontal_center, vertical_center]
    bottom_right = [horizontal_center, vertical_center]

    current_frame = 0

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        current_frame +=1
        # Capture frame-by-frame
        ret, frame = cap.read()
        QR_frame = QR_Reader(frame, ret)
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        # resize the frame, blur it, and convert it to the HSV
        # color space


        #circles, circle_edges = balls(frame)
        keypoints = balls2(frame)
        #centerPoint = bigEgg(frame)
        lines, course_edges, mask_img = find_edges(frame)

        ## Only find course during startup, otherwise we expect the course to be static.
        if FIND_COURSE:
            update_course_edges(lines, horizontal_center, vertical_center)
            ##find_cross(frame)
            cv2.putText(frame,'FINDING COURSE BOUNDARIES',
                [horizontal_center, vertical_center],
                font,
                fontScale,
                fontColor,
                thickness,
                lineType)
            if current_frame > REFRESH_BOUND_FRAME:
                FIND_COURSE = False



        #draw_balls(circles, frame)
        origin = find_robot_origin(frame)
        dir_vector = find_robot_direction(frame)
        if origin is not None:
            for p in origin:
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
                break
        if dir_vector is not None:
            for p in dir_vector:
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
                break
    

        #centerPoint = bigEgg(frame)
        #draw_eggs(centerPoint,frame)
        draw_balls2(keypoints, frame)
        draw_course(frame)
        #draw_cross(frame)

        


        stack1 = np.concatenate((frame, frame), axis=0)
        stack2 = np.concatenate((frame, frame), axis=0)
        stack3 = np.concatenate((stack1, stack2), axis=1)
        cv2.imshow('blur', stack3)

        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()



start()




