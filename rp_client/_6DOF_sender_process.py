""" Class takes the sensor, runs a loop taking input to the buffer till it gets a set number of points.
 After that the buffer if flushed into new process """
from _6DOF_sensor import _6DOF_sensor
# import numpy as np
import pandas as pd  # for influx sender class
#import threading  # for parallelism
from time import sleep  # for basic pausing
import datetime  # for getting measurement time in microseconds
# from queue import Queue  # for safe dataflow between threads
# from queue import Empty
import Database_sender
import multiprocessing as mp
from math import sqrt
# Global constants
packet_length = 5000


def send_process(list_in, db_host_address_in, names_in, db_in, sensor_in):
    if list_in:
        #print('send process started')
        client = Database_sender.Database_sender(host=db_host_address_in)
        df_temp = pd.DataFrame(list_in, columns=names_in)  # .set_index(self.names[0])#, index=[self.names[0]])
        df_temp.set_index(names_in[0], inplace=True)
        df_temp.index = pd.to_datetime(df_temp.index, utc=True)  # , unit='us')

        client.send_measurement(df=df_temp,  # pd.DataFrame(list_of_lists, columns=self.names).set_index(self.names[0]),
                            db_name=db_in, measurement_name=sensor_in + '_raw')


class Movement_sensing:
    def __init__(self):
        #self.laying_event = event_in
        self.sensor_not_moving = False  # <-------------- I have to provide an algorithm to determine that state
        self.f_gain = 0.97  # 1 - averaging filter gain y_k=y_k-1*f_gain + x_k*(1-f_gain) NEVER OVERWRITE
        self.f_gain_2 = 1-self.f_gain
        self.y_k = 0
        self.time_hysteresis_lay = 3  # ms - 2 seconds without move to call stop flag
        self.timer_lay = datetime.datetime.utcnow().timestamp()  # us
        self.time_hysteresis_move = 10 / 1000  # s - 10 ms of movement to call start flag
        self.timer_move = datetime.datetime.utcnow().timestamp() # us
        self.g_hysteresis_width = 0.25  # m/s^2  - width of hysteresis NEVER OVERWRITE
        self.g_hysteresis_set_point = 9.81  # m/s^2  - set-point NEVER OVERWRITE
        self.high_border = self.g_hysteresis_set_point + self.g_hysteresis_width
        self.low_border = self.g_hysteresis_set_point - self.g_hysteresis_width
        print('high_border :',self.high_border)
        print('low_border :',self.low_border)

    def point(self, g_tot_in):
        # calculate g_tot
        g_tot = sqrt(g_tot_in[3]**2+g_tot_in[4]**2+g_tot_in[5]**2)
        self.y_k = self.y_k*self.f_gain + g_tot*self.f_gain_2
        #print(self.y_k)
        time_now = datetime.datetime.utcnow().timestamp()
        # check movement conditions
        if self.y_k > self.high_border or self.y_k < self.low_border:
            # movement sensing

            # check movement timer
            if self.timer_move+self.time_hysteresis_move < time_now:
                #print(self.y_k)
                if self.sensor_not_moving:
                    print('found movement')
                    #self.laying_event.set()
                self.sensor_not_moving = False
                
            # zero laying timer
            self.timer_lay = time_now
        else:
            # no movement sensing

            # check laying timer
            if self.timer_lay + self.time_hysteresis_lay < time_now:
                if not self.sensor_not_moving:
                    print('lost movement')
                    #self.laying_event.clear()
                self.sensor_not_moving = True

            # zero move timer
            self.timer_move = time_now
        
        return self.sensor_not_moving
    def get_g(self):
        return self.y_k

class _6DOF_sender_process(mp.Process):
    """Class spawns 2 threads. One for acquiring data from sensor. Second for sending data"""

    def __init__(self, start_time_in, event_list, db_name_in, db_host_address, sensor_in=''):
        """

        :param start_time_in: Microsecond utc time of measurement start.
        :param event_list: Thread events for control of thread work. Dict of {name:thread}
        :param db_name_in: Name od InfluxDb database to send to.
        :param db_host_address: IP address of InfluxDb server.
        :param sensor_in: Name of the used sensor. Has to be one of sensors implemented in _6DOF_sensor.py
        """
        mp.Process.__init__(self)
        self.events = event_list
        self.internal_events = {'dequeueing': mp.Event()} # Dequeueing flag (Only one)
        # Connect_to_sensor
        self.sensor = _6DOF_sensor(sensor_type=sensor_in)
        self.start_time = start_time_in
        #self.send_data = False
        self.db_name = db_name_in
        # self.send_scanner = False
        # Create queue object
        self.q = mp.Queue()
        #self.deq = False  
        # Initialize dataframe column names
        self.names = ['time_abs', 'time_rel', 'gyro_X', 'gyro_Y', 'gyro_Z', 'acc_X', 'acc_Y', 'acc_Z']
        # Create DB client
        self.db_client = Database_sender.Database_sender(host=db_host_address)
        self.db_address = db_host_address
        if not self.db_client.check_connection():
            raise RuntimeError('No dataframe connection')
        # Create collector thread
        self.collector = mp.Process(target=self.sensor_data_collector)
        # Create sender thread
        self.sender = mp.Process(target=self.sensor_data_sender)
        self.movement_sensor = Movement_sensing()#event_list['sensor laying'])
        self.scanner_laying = False
        self.g_tot=0.0
        print('Sensor initialised: ',sensor_in)

    def run(self):
        print('Sensor starting: ',self.sensor.sensor_type)
        while not self.events['stop main'].is_set():
            if self.events['start collector'].is_set() and not self.events['running collector'].is_set():
                print(self.sensor.sensor_type,' Sensor collector starting')
                self.collector = mp.Process(target=self.sensor_data_collector)
                self.collector.start()  # Start collector thread
                self.events['start collector'].clear()  # Clear flag for later use
            if self.events['start sender'].is_set() and not self.events['running sender'].is_set():
                print(self.sensor.sensor_type,' Sensor sender starting')
                self.sender = mp.Process(target=self.sensor_data_sender)
                self.sender.start()  # Start sender thread
                self.events['start sender'].clear()  # Clear flag for later use
            sleep(0.02)
        self.events['stop main'].clear()  # Clear flag for later use

        # stop both sub-worker threads
        self.events['stop collector'].set()  # End data collection
        while self.collector.is_alive():  # Wait for thread to stop
            sleep(0.01)
        self.events['stop sender'].set()  # End data sending
        while self.sender.is_alive():  # Wait for thread to stop
            sleep(0.01)
    

    def sensor_data_collector(self):
        """Thread for collecting data from sensor"""
        self.events['running collector'].set()
        print(self.sensor.sensor_type,' Starting collector')
        list_of_meas = []
        while not self.events['stop collector'].is_set():
            time_now = datetime.datetime.utcnow()
            # print(time_now)
            time_rel = time_now.timestamp() - self.start_time
            # get sensor output
            sens_out = self.sensor.get_sensor_data()
            # determine whether if the scanner is moving
            scanner_laying_temp = self.movement_sensor.point(sens_out)
            if self.scanner_laying != scanner_laying_temp:
                self.scanner_laying = scanner_laying_temp
                if scanner_laying_temp:
                    print(self.sensor.sensor_type,' Setting laying event: ')
                    self.events['sensor laying'].set()
                else:
                    print(self.sensor.sensor_type,' Clearing laying event: ')
                    self.events['sensor laying'].clear()
                    
                    
            #self.g_tot = self.movement_sensor.y_k
            if self.events['send data'].is_set():
                list_of_meas.append([time_now, time_rel] + sens_out)

            if not self.internal_events['dequeueing'].is_set() and len(list_of_meas) > 10:
                #print('enqueing_data')
                self.q.put(list_of_meas)
                list_of_meas = []
        # send remaining points
        if len(list_of_meas) > 0:
            self.q.put(list_of_meas)
            list_of_meas = []
        self.events['stop collector'].clear()  # Clear flag for later use
        print(self.sensor.sensor_type,' Stopping collector')
        sleep(0.02)
        self.events['running collector'].clear()

    def sensor_data_sender(self):
        """Thread for sending data to server database"""
        self.events['running sender'].set()
        print(self.sensor.sensor_type,' Starting sender')
        # Check if db exists
        if not self.db_client.check_connection():
            raise RuntimeError('DB no connection')
        # Check if database exists
        if not self.db_client.check_database(self.db_name):
            raise RuntimeError('No such database')

        def rest_sender(length):

            self.internal_events['dequeueing'].set()
            list_of_lists2 = []

            for i in range(length):
                try:
                    list_of_lists2 = list_of_lists2 + self.q.get(block=True)
                except Empty:
                    sleep(0.01)

            self.internal_events['dequeueing'].clear()
            p2 = mp.Process(target=send_process,
                            args=(list_of_lists2, self.db_address, self.names, self.db_name, self.sensor.sensor_type))
            p2.start()
            p2.join()

        list_of_lists = []
        p = mp.Process(target=send_process,
                       args=(list_of_lists, self.db_address, self.names, self.db_name, self.sensor.sensor_type))
        p.start()
        while not self.events['stop sender'].is_set() or self.events[
            'running collector'].is_set():  # Run while data is collected
            #print('sender_loop_call')
            elements = self.q.qsize()
            if elements > 20:
                try:
					self.internal_events['dequeueing'].set()
					for i in range(elements):
						list_of_lists = list_of_lists + self.q.get(block=True)
                    self.internal_events['dequeueing'].clear()
                except Empty:
                    sleep(0.01)

                #print(len(list_of_lists))
                if len(list_of_lists) > packet_length and not p.is_alive():
                    #print('calling_the sender')
                    p = mp.Process(target=send_process, args=(
                        list_of_lists, self.db_address, self.names, self.db_name, self.sensor.sensor_type))
                    p.start()
                    list_of_lists = []
            else:
                sleep(0.05)
        p.join()
        print(self.sensor.sensor_type, ' Sending rest of queue data')
        if self.q.qsize() > 0:
            rest_sender(self.q.qsize())
        self.events['stop sender'].clear()
        print(self.sensor.sensor_type,' Stopping sender')
        self.events['running sender'].clear()


if __name__ == '__main__':
    """Test code:
      1. connect to selected sensor
      2. create and connect to database
      3. send sensor data for a few seconds
      """
    start_time = datetime.datetime.utcnow().timestamp()
    thread_events = {'stop collector': mp.Event(),
                     'stop sender': mp.Event(),
                     'stop main': mp.Event(),
                     'start collector': mp.Event(),
                     'start sender': mp.Event(),
                     'running collector': mp.Event(),
                     'running sender': mp.Event(),
                     'send data': mp.Event()}
    #sensor = 'MPU_9255'
    sensor = 'ISM_330'
    db_name = 'scan_sensor_test'
    db_ip_address = '192.168.1.47'
    sensor_thread = _6DOF_sender_process(start_time_in=start_time,
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
    thread_events['send data'].set()
    thread_events['start sender'].set()
    print('Calling start collector')
    thread_events['start collector'].set()
    for i in range(2):
        sleep(5)
    print('Calling stop collector')
    thread_events['stop collector'].set()
    # sleep(1)
    # print('Calling start collector')
    # thread_events['start collector'].set()
    # sleep(1)
    print('Calling stop main')
    thread_events['stop main'].set()
