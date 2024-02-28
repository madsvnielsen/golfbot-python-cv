import cv2
import numpy as np
from collections import deque
from imutils.video import VideoStream
import threading
import sys
import math



## BALL DETECTION
GAUSSIAN_BLUR = (37,37)
EDGE_CANNY_THRESHOLD = 60
EDGE_ACCUMULATOR_THRESHOLD = 60
HOUGH_DP = 3.8
HOUGH_MIN_DISTANCE = 50
HOUGH_MIN_RADIUS = 5
HOUGH_MAX_RADIUS = 25

## BOUNDARY DETECTION
BOUND_DP = 10
BOUND_ANGLE_RES = np.pi/180
BOUND_THRESH = 30
BOUND_MINLEN = 50
BOUND_MAXGAP = 5




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


def balls(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
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




def start():
    cap = cv2.VideoCapture(1)
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (10,500)
    fontScale              = 1
    fontColor              = (255,0,0)
    thickness              = 3
    lineType               = 2

    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        # resize the frame, blur it, and convert it to the HSV
        # color space


        circles, circle_edges = balls(frame)
        lines, course_edges, mask_img = find_edges(frame)

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
        if lines is not None:
            for points in lines:
                x1,y1,x2,y2=points[0]
                delta_y = y2-y1
                delta_x = x2-x1
                slope = math.atan2(delta_y, delta_x)
                cv2.line(frame,(x1,y1),(x2,y2),(0,255,0),2)

                slope_type = "Unknown"

                if 1.3 < abs(slope) < 1.6:
                    slope_type = "Vertical"

                if -0.5 < slope < 0.5:
                    slope_type = "Horizontal"

                cv2.putText(frame,str(slope_type) + ": " + str(slope),
                    (x1,y1),
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)


        stack1 = np.concatenate((cv2.cvtColor(circle_edges, cv2.COLOR_GRAY2BGR), frame), axis=0)
        stack2 = np.concatenate((frame, mask_img), axis=0)
        stack3 = np.concatenate((stack1, stack2), axis=1)
        cv2.imshow('blur', stack3)

        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


threading.Thread(target=monitor_cmd).start()
start()
