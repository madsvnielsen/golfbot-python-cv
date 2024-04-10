import socket
import threading
import curses

bind_ip = "192.168.4.170"  # Replace this with your own IP address
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

def clientHandler(client_socket):


    # Initialize curses
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)



    # Define the function to send key inputs to the client


    try:
        # Your code for displaying instructions goes here
        stdscr.clear()
        stdscr.addstr(0, 0, 'Use Arrows to navigate your EV3-vehicle')
        stdscr.addstr(1, 0, 'Pause your vehicle with key <p>')
        stdscr.addstr(2, 0, 'Terminate with key <q>')
        stdscr.refresh()

        # Continuously read key inputs and send them to the client
        while True:
            c = stdscr.getch()
            if c in [ord('q'), 27]:
                send_key_input(c, client_socket)
                break
            elif c in [ord('p'),
                       curses.KEY_RIGHT, curses.KEY_LEFT, curses.KEY_UP, curses.KEY_DOWN, curses.KEY_BACKSPACE]:
                send_key_input(c, client_socket)
    finally:
        # Cleanup curses
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

        # Close the client socket
        client_socket.close()

while True:
    client, addr = server.accept()
    print("Client connected " + str(addr))
    client.send(str("k").encode())
    #client_handler = threading.Thread(target=clientHandler, args=(client,))
    #client_handler.start()
