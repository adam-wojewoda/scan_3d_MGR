# Scanner tcp server library code
# This code takes care of:
#   1. Sharing gui commands with client
#   2. Getting client state for gui to use

import socket


class TcpServer:
    def __init__(self):
        HOST = ''  # Symbolic name meaning all available interfaces
        PORT = 50007  # Arbitrary non-privileged port
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()
        print('Connected by', addr)

        while 1:
            data = conn.recv(1024)
            if not data: break
            conn.sendall(data)
        conn.close()


# Test specific code

# Test calling
if __name__ == '__main__':
    print('Lack of testing code')
