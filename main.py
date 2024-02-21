import cv2
import numpy as np
from collections import deque
from imutils.video import VideoStream





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
    hist = cv2.equalizeHist(gray)
    blurred = cv2.GaussianBlur(hist, (37, 37), cv2.BORDER_DEFAULT)
    cimg = cv2.cvtColor(blurred,cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(blurred,cv2.HOUGH_GRADIENT,1,100,
                            param1=60,param2=40,minRadius=5,maxRadius=300)


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



    horizontalStack = np.hstack((frame, cimg))
    cv2.imshow('blur', horizontalStack)

    if cv2.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
