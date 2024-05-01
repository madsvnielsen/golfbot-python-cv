import socket
import threading
import curses
import random
import time


class Server:
    #bind_ip = "192.168.4.170"  # Replace this with your own IP address
    bind_ip = "0.0.0.0"  # Replace this with your own IP address

    bind_port = 27700  # Feel free to change this port

    def __init__(self) -> None:
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.bind_ip, self.bind_port))
        self.server.listen(5)
        print("Server is listening on %s:%d" % (self.bind_ip, self.bind_port))
        self.client, self.addr = self.server.accept()
        
    

    def send_key_input(self, key):
        
        print("Client connected " + str(self.addr))
        self.client.send(str(key).encode())
    '''
    def recvCommand(command):
        self.clientHandler(client,command)

    def clientHandler(client_socket,command):
            send_key_input(command)
            client_socket.close()
            


while True:
    client, addr = server.accept()
    print("Client connected " + str(addr))
    client_handler = threading.Thread(target=clientHandler, args=(client,))
    client_handler.start()
'''