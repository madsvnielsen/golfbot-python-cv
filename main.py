import cv2
import numpy as np
from collections import deque
from imutils.video import VideoStream
import threading
import sys





GAUSSIAN_BLUR = (37,37)

EDGE_CANNY_THRESHOLD = 60
EDGE_ACCUMULATOR_THRESHOLD = 40

HOUGH_DP = 0.8
HOUGH_MIN_DISTANCE = 100
HOUGH_MIN_RADIUS = 5
HOUGH_MAX_RADIUS = 300



def monitor_cmd():
    global GAUSSIAN_BLUR, EDGE_ACCUMULATOR_THRESHOLD,  EDGE_CANNY_THRESHOLD, HOUGH_DP, HOUGH_MIN_RADIUS, HOUGH_MIN_DISTANCE, HOUGH_MAX_RADIUS
    while True:
        print("""
        CURRENT SETTINGS
        **POST PROCESSING**
        BLUR = %s

        **EDGE DETECTION**
        CANNY = %s
        ACCUM = %s

        **CIRCLE DETECTION**
        DP = %s
        MINDIST = %s
        MINRAD = %s
        MAXRAD = %s

        Type SETTING <value> to change.

        """
        % (GAUSSIAN_BLUR[0], EDGE_CANNY_THRESHOLD, EDGE_ACCUMULATOR_THRESHOLD, HOUGH_DP, HOUGH_MIN_DISTANCE, HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS))
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










def start():
    cap = cv2.VideoCapture(0)
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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hist = cv2.GaussianBlur(gray, GAUSSIAN_BLUR, cv2.BORDER_DEFAULT)
        blurred = cv2.equalizeHist(hist)

        edges = cv2.Canny(blurred,EDGE_CANNY_THRESHOLD,EDGE_ACCUMULATOR_THRESHOLD)
        cimg = cv2.cvtColor(blurred,cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,
        HOUGH_DP,HOUGH_MIN_DISTANCE,
        param1=EDGE_CANNY_THRESHOLD,
        param2=EDGE_ACCUMULATOR_THRESHOLD,
        minRadius=HOUGH_MIN_RADIUS,
        maxRadius=HOUGH_MAX_RADIUS)





        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                # draw the outer circle
                cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)
                cv2.putText(frame,'Dog',
                    (i[0],i[1]),
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)

                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                cv2.putText(cimg,'Dog',
                    (i[0],i[1]),
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)


        stack1 = np.concatenate((frame, cimg), axis=0)
        stack2 = np.concatenate((cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR), cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)), axis=0)
        vstack = np.concatenate((stack1, stack2), axis=1)
        cv2.imshow('blur', vstack)

        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


threading.Thread(target=monitor_cmd).start()
start()
