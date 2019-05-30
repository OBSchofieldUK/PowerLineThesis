import socket

INTERNAL = None
EXTERNAL = None
LOCALHOST_IP = "127.0.0.1"

def recieve():
    global EXTERNAL, INTERNAL
    UDP_IP = "0.0.0.0"
    UDP_PORT = 14550

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes

        if(INTERNAL == None and addr[0]== LOCALHOST_IP):
            INTERNAL = addr
            print("Internal Registerd: ", INTERNAL)
        if(EXTERNAL == None and not addr[0] == LOCALHOST_IP):
            EXTERNAL = addr
            print("EXTERNAL registered: ", EXTERNAL)

        if(addr == INTERNAL and not EXTERNAL == None):
            sock.sendto(data, EXTERNAL)
            print("Mesage from: ", INTERNAL, " To ", EXTERNAL)
        elif(addr == EXTERNAL and not INTERNAL == None):
            sock.sendto(data, INTERNAL)
            print("Mesage from: ", EXTERNAL, " To ", INTERNAL)
        else:
            print("received message From:", addr)
recieve()
