
import socket

from ev3dev2.auto import *
from ev3dev2.motor import MoveTank
import time

m = Motor(OUTPUT_C)
motors= MoveTank(OUTPUT_A,OUTPUT_B)

gyro =GyroSensor(INPUT_1)
motors.gyro =gyro
motors.gyro.calibrate()
#target_host = "192.168.4.170" # Change this to the IP address of your server
#target_host = "172.20.10.13"
target_host = "192.168.75.48"
target_port = 27700 # Change this to the port of your server

# create a socket
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(5)
# connect to the server
client.connect((target_host, target_port))


def start() -> None:
    m.on(100)

def moveBackward() -> None:
    motors.on_for_seconds(20, 20, 0.2)

def moveFoward() -> None:
    motors.on_for_seconds(-20, -20, 0.75)

def turnRight(secs) -> None:
    motors.on(-10,10)
    #motors.on_for_seconds(-10, 10, secs)

def turnLeft(secs) -> None:
    motors.on(10,-10)
    #motors.on_for_seconds(10, -10, secs)
def dumpBalls() -> None:
    m.stop()
    m.on_for_seconds(-40, 10)
    m.on(100)

def stop() -> None:
    m.stop()
    motors.stop()


def react(command):
    global speed, turn
    args = command.split(" ")
    c = args[0]
    if c in [ord('q'), 27, ord('p')]:
        return
    elif c == "left": #leftkey
      turnLeft(float(args[1]))
    elif c == "right": #rightkey
       turnRight(float(args[1]))
    elif c == "forward": # up key
        moveFoward()
    elif c == "back": #down key
        moveBackward()
    elif c == "k":
        dumpBalls()
    elif c == "stop":
        stop()
    elif c == "start":
        start()
# receive
while True:
    try:
        # receive data from the server
        response = client.recv(4096)
        if not response:  # If the response is empty, the server closed the connection
            print("Server closed the connection.")
            break
        # decode and print the received data
        print(response.decode())
        react(response.decode())
    except socket.timeout:  # Handle timeout
        pass
# send

