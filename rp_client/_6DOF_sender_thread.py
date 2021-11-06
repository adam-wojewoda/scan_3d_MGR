""" Class takes the sensor, runs a loop taking input to the buffer till it gets a set number of points.
 After that the buffer if flushed into new process """
from _6DOF_sensor import _6DOF_sensor
import pandas as pd  # for influx sender class
import threading  # for parallelism
from time import sleep  # for basic pausing (reliving CPU)
import datetime  # for getting measurement time in microseconds
from queue import Queue  # for safe dataflow between threads


class _6DOF_sender_thread(threading.Thread):
    """Class spawns 2 threads. One for acquiring data from sensor. Second for sending data"""

    def __init__(self, start_time, event_list, sensor=''):
        threading.Thread.__init__(self)
        self.events = event_list
        # Connect_to_sensor
        self.sensor = _6DOF_sensor(sensor_type=sensor)
        self.start_time = start_time

        # Create queue object
        self.q = Queue()
        # Initialize dataframe column names
        self.names = ['time_abs', 'time_rel', 'gyro_Z', 'gyro_Y', 'gyro_Z', 'acc_X', 'acc_Y', 'acc_Z']
        # Create collector thread
        self.collector = threading.Thread(target=self.sensor_data_collector)
        # Create sender thread
        self.sender = threading.Thread(target=self.sensor_data_sender)

    def run(self):
        while not self.events['stop main'].is_set():  # clear() set
            sleep(0.05)
        # stop both sub-worker threads
        self.events['stop collector'].set()
        sleep(0.1)
        self.events['stop sender'].set()
        sleep(0.1)

    def sensor_data_collector(self):
        while not self.events['stop collector'].is_set():
            time_now = datetime.datetime.utcnow().microsecond
            time_rel = time_now - self.start_time

            self.q.put(pd.DataFrame([time_now, time_rel] + self.sensor.get_sensor_data(), columns=self.names))

    def sensor_data_sender(self):
        while not self.events['stop sender'].is_set():
            pass
