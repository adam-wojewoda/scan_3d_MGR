# Code for communication with TCP server on PC
# This code takes care of:
#   1. Taking commands from main server for main process to take

# Echo client program
import socket


class TCP_socket_client:
    def __init__(self, host_in, port_in):
        self.list_sockets = []  # [{message : xxx, socket : xxx}, ...]
        self.host = host_in
        self.port = port_in
        self.available_messages = {
            'inf sensors error': b'sens_err',
            'inf sensors ok': b'sens_ok',
            'inf laser error': b'laser_err',
            'inf laser ok': b'laser_ok',
            'inf laser working': b'laser_working',
            'inf laser waiting': b'laser_waiting',
            'inf laser put down': b'laser_put_down',
            'inf laser picked up': b'laser_picked_up',
            'que get db name': b'get_db_name',
            'que get command': b'get_command',  # <-------- very important one (aks what to do)
        }
        self.available_commands = {
            'com start sensors': b'sens_start',
            'com stop sensors': b'sens_stop',
            'com start laser': b'laser_start',
            'com stop laser': b'laser_stop',
            'com get sensor status': b'get_sens_stat',
            'com get laser status': b'get_laser_stat',
            'com get laser working': b'get_laser_working',
            'com new db name av': b'new_db_name_av',
            'res message received': b'message_received',
            'com carry on': b'carry_on'  # <-------- sent as info, that everything is ok
        }
        pass

    def send_message(self, message):
        # create socket
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # connect my dear socket
        soc.connect((self.host, self.port))
        # send the message to server
        soc.sendall(self.available_messages[message])
        # add it to list of active sockets
        self.list_sockets.append({'message': message, 'socket': soc})

    def check_response(self):
        # check if we have any sockets to read
        if len(self.list_sockets) > 0:
            response = self.list_sockets[0]['socket'].recv(1024)
            message = self.list_sockets[0]['message']
            self.list_sockets[0]['socket'].close()
            del self.list_sockets[0]
            return {'message': message, 'response': response}
        else:
            # no sockets to read, return None
            return None


# Test calling
if __name__ == '__main__':
    # Make sure You have the test server running

    def tester(sc, mess_in):
        print('Sending message: ', mess_in)
        socket_client.send_message(mess_in)
        print('Server response: ', sc.check_response()['response'])
        print(' ')


    socket_client = TCP_socket_client('localhost', 50007)

    # 1. 'inf sensors error': b'sens_err'
    tester(socket_client, 'inf sensors error')
    # 2. 'inf sensors ok': b'sens_ok'
    tester(socket_client, 'inf sensors ok')
    # 3. 'inf laser error': b'laser_err'
    tester(socket_client, 'inf laser error')
    # 4. 'inf laser ok': b'laser_ok'
    tester(socket_client, 'inf laser ok')
    # 5. 'inf laser working': b'laser_working'
    tester(socket_client, 'inf laser working')
    # 6. 'inf laser waiting': b'laser_waiting'
    tester(socket_client, 'inf laser waiting')
    # 7. 'inf laser put down': b'laser_put_down'
    tester(socket_client, 'inf laser put down')
    # 8. 'inf laser picked up': b'laser_picked_up'
    tester(socket_client, 'inf laser picked up')
    # 9. 'que get db name': b'get_db_name'
    tester(socket_client, 'que get db name')
    # 10.'que get command': b'get_command'
    tester(socket_client, 'que get command')
