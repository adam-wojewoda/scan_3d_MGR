""" Class takes the sensor, runs a loop taking input to the buffer till it gets a set number of points.
 After that the buffer if flushed into new process """
from _6DOF_sensor import _6DOF_sensor
#import numpy as np
import pandas as pd  # for influx sender class
import threading  # for parallelism
from time import sleep  # for basic pausing (reliving CPU)
import datetime  # for getting measurement time in microseconds
from queue import Queue  # for safe dataflow between threads
from queue import Empty
import Database_sender

# Global constants
packet_length = 100


class _6DOF_sender_thread(threading.Thread):
    """Class spawns 2 threads. One for acquiring data from sensor. Second for sending data"""

    def __init__(self, start_time_in, event_list, db_name_in, db_host_address, sensor_in=''):
        """

        :param start_time_in: Microsecond utc time of measurement start.
        :param event_list: Thread events for control of thread work. Dict of {name:thread}
        :param db_name_in: Name od InfluxDb database to send to.
        :param db_host_address: IP address of InfluxDb server.
        :param sensor_in: Name of the used sensor. Has to be one of sensors implemented in _6DOF_sensor.py
        """
        threading.Thread.__init__(self)
        self.events = event_list
        # Connect_to_sensor
        self.sensor = _6DOF_sensor(sensor_type=sensor_in)
        self.start_time = start_time_in
        self.send_data = False
        self.db_name = db_name_in
        # self.send_scanner = False
        # Create queue object
        self.q = Queue()
        # Initialize dataframe column names
        self.names = ['time_abs', 'time_rel', 'gyro_Z', 'gyro_Y', 'gyro_Z', 'acc_X', 'acc_Y', 'acc_Z']
        # Create DB client
        self.db_client = Database_sender.Database_sender(host=db_host_address)
        if not self.db_client.check_connection():
            raise RuntimeError('No dataframe connection')
        # Create collector thread
        self.collector = threading.Thread(target=self.sensor_data_collector)
        # Create sender thread
        self.sender = threading.Thread(target=self.sensor_data_sender)

    def run(self):
        while not self.events['stop main'].is_set():
            if self.events['start collector'].is_set() and not self.collector.isAlive():
                self.collector = threading.Thread(target=self.sensor_data_collector)
                self.collector.start()  # Start collector thread
                self.events['start collector'].clear()  # Clear flag for later use
            if self.events['start sender'].is_set() and not self.sender.isAlive():
                self.sender = threading.Thread(target=self.sensor_data_sender)
                self.sender.start()  # Start sender thread
                self.events['start sender'].clear()  # Clear flag for later use
            sleep(0.02)
        self.events['stop main'].clear()  # Clear flag for later use

        # stop both sub-worker threads
        self.events['stop collector'].set()  # End data collection
        while self.collector.isAlive():  # Wait for thread to stop
            sleep(0.01)
        self.events['stop sender'].set()  # End data sending
        while self.sender.isAlive():  # Wait for thread to stop
            sleep(0.01)

    def sensor_data_collector(self):
        """Thread for collecting data from sensor"""
        print('Starting collector')
        while not self.events['stop collector'].is_set():
            time_now = datetime.datetime.now()
            #print(time_now)
            time_rel = time_now.timestamp() - self.start_time
            if self.send_data:
                self.q.put([time_now, time_rel] + self.sensor.get_sensor_data())
        self.events['stop collector'].clear()  # Clear flag for later use
        print('Stopping collector')

    def sensor_data_sender(self):
        """Thread for sending data to server database"""
        print('Starting sender')
        # Check if db exists
        if not self.db_client.check_connection():
            raise RuntimeError('DB no connection')
        # Check if database exists
        if not self.db_client.check_database(self.db_name):
            raise RuntimeError('No such database')

        def sender_proto(length):
            print(length)
            list_of_lists = []
            for i in range(length):
                try:
                    list_of_lists.append(self.q.get(block=False))
                except Empty:
                    sleep(0.001)
                    
            #print(list_of_lists)
            df_temp = pd.DataFrame(list_of_lists, columns=self.names)#.set_index(self.names[0])#, index=[self.names[0]])
            df_temp[self.names[0]] = pd.DatetimeIndex(df_temp[self.names[0]])
            #df_temp.set_index(self.names[0])
            print(df_temp.set_index(self.names[0]))
            
            self.db_client.send_measurement(df=df_temp.set_index(self.names[0]),#pd.DataFrame(list_of_lists, columns=self.names).set_index(self.names[0]),
                                            db_name=self.db_name, measurement_name=self.sensor.sensor_type + '_raw')

        # main sender loop
        while not self.events['stop sender'].is_set() or self.collector.isAlive():  # Run while data is collected
            if self.q.qsize() > packet_length:
                sender_proto(length=packet_length)
            sleep(0.05)
        print('sending remining queue data')
        if self.q.qsize()>0:
            sender_proto(self.q.qsize())
        self.events['stop sender'].clear()
        print('Stopping sender')


if __name__ == '__main__':
    """Test code:
      1. connect to selected sensor
      2. create and connect to database
      3. send sensor data for a few seconds
      """
    start_time = datetime.datetime.now().timestamp()
    thread_events = {'stop collector': threading.Event(),
                     'stop sender': threading.Event(),
                     'stop main': threading.Event(),
                     'start collector': threading.Event(),
                     'start sender': threading.Event()}
    sensor = 'MPU_9255'
    # sensor = 'ISM_330'
    db_name = 'scan_sensor_test'
    db_ip_address = '192.168.10.15'
    sensor_thread = _6DOF_sender_thread(start_time_in=start_time,
                                        event_list=thread_events,
                                        db_name_in=db_name,
                                        db_host_address=db_ip_address,
                                        sensor_in=sensor)

    print('Testing connection')
    print('Server IP: ' + db_ip_address)
    if not sensor_thread.db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')
    if not sensor_thread.db_client.check_database(db_name):
        sensor_thread.db_client.create_database(db_name)  # Create database container on server
    print('Calling start sensor thread')
    sensor_thread.start()  # Start the thread
    print('Calling start sender')
    sensor_thread.send_data = True
    thread_events['start sender'].set()
    print('Calling start collector')
    thread_events['start collector'].set()
    sleep(0.1)
    print('Calling stop collector')
    thread_events['stop collector'].set()
    sleep(1)
    print('Calling start collector')
    thread_events['start collector'].set()
    sleep(0.1)
    print('Calling stop main')
    thread_events['stop main'].set()
