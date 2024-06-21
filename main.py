import sys
from OpenCV.VideoTest import start_video_test
from OpenCV.RobotController import start_robot_controller


args = sys.argv
if len(args) > 1:
    if args[1] == "test":
        start_video_test()
        sys.exit()

start_robot_controller()

