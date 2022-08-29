# Scanner tcp server library code
# This code takes care of:
#   1. Sharing gui commands with client
#   2. Getting client state for gui to use

import socket
import threading
from queue import Queue
from time import sleep


class TcpServer(threading.Thread):
    def __init__(self, commands_q: Queue, messages_q: Queue, events_in):
        threading.Thread.__init__(self)
        self.connection_threads = Queue()
        self.events_list = events_in
        self.commands_queue = commands_q
        self.messages_queue = messages_q
        self.host = ''  # Symbolic name meaning all available interfaces
        self.port = 50007  # Arbitrary non-privileged port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.host, self.port))

        """I know I should import these from some other file, but i'll do it later when it works properly"""
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

    def operate_connection(self, conn_in):
        print('new connection')
        while 1:
            data = conn_in.recv(1024)
            # try to interpret the connection

            if not data: break  # end of connection
            conn_in.sendall(data)
        conn_in.close()
        pass

    def close_threads(self):
        print('threads closer started')
        while not (self.events_list['stop_server'].is_set() and self.connection_threads.empty()):
            if not self.connection_threads.empty():
                tmp = self.connection_threads.get()
                tmp.join()
                print('thread closed')
            sleep(0.1)

    def run(self):
        print('starting the server')
        self.events_list['stop_server'].clear()
        print('creating thread closer')
        closer_thread = threading.Thread(target=self.close_threads)
        print('starting thread closer')
        closer_thread.start()
        print('calling listen loop')
        while not self.events_list['stop_server'].is_set():

            self.s.settimeout(0.1)
            try:
                self.s.listen(1)
                conn, addr = self.s.accept()
                print('Connected by', addr)
                t = threading.Thread(target=self.operate_connection, args=[conn])
                t.start()
                self.connection_threads.put(t)
            except socket.timeout:
                pass
        print('no more listening, waiting for connections to close')
        closer_thread.join()
        print('server stopped')


# Test calling
if __name__ == '__main__':
    commands_q=Queue()
    messages_q=Queue()
    events_in={'stop_server': threading.Event()}
    events_in['stop_server'].clear()

    server = TcpServer(commands_q,messages_q,events_in)
    server.start()
    sleep(20)
    print('calling stop command')
    events_in['stop_server'].set()
