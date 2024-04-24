import cv2

import main
import Server1
def start():
    start_generator=main.start()

    for keypoints in start_generator:
        balls = keypoints
        print('cordinates'+balls)

    robot=cv2.KeyPoint(100, 200, _size=10, _angle=0, _response=0, _octave=0, _class_id=-1)
    Server1.clientHandler()

