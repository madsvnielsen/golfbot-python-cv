import socket
import threading
import curses
import random
import time

#bind_ip = "192.168.4.170"  # Replace this with your own IP address
bind_ip = "172.20.10.13"  # Replace this with your own IP address

bind_port = 27700  # Feel free to change this port

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((bind_ip, bind_port))
server.listen(5)
print("Server is listening on %s:%d" % (bind_ip, bind_port))

def send_key_input(key, client_socket):
    try:
        client_socket.send(str(key).encode())
    except Exception as e:
        print("Error sending key input:", e)
def recvCommand(command):
    clientHandler(client,command)
def clientHandler(client_socket,command):

        send_key_input(command)
        client_socket.close()

        
while True:
    client, addr = server.accept()
    print("Client connected " + str(addr))
    client_handler = threading.Thread(target=clientHandler, args=(client,))
    client_handler.start()
