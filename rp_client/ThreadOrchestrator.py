# Code for main loop of data acquisition from scanner
# This code takes care of:
#   1. Calling all scanner side processes
#   2. Sending the data to database
#   3. Close the program after finishing

import datetime
import _6DOF_sender_thread
from LASER_scanner import LASER_scanner_thread
from LASER_scanner import Measurement_button
import multiprocessing as mp
import threading
from time import sleep
import tkinter as tk


class ThreadOrchestrator(threading.Thread):
    def __init__(self, start_time_in, server_ip, scanner_ip, db_name, events_in: {str: threading.Event}):
        """Create all the threads, prepare for some work"""
        threading.Thread.__init__(self)
        self.scanning_command = False
        self.trigger = Measurement_button.Measurement_button(11)
        self.main_events = events_in
        self.sensor_event_lists = [{'stop collector': mp.Event(),
                                    'stop sender': mp.Event(),
                                    'stop main': mp.Event(),
                                    'start collector': mp.Event(),
                                    'start sender': mp.Event(),
                                    'running collector': mp.Event()},
                                   {'stop collector': mp.Event(),
                                    'stop sender': mp.Event(),
                                    'stop main': mp.Event(),
                                    'start collector': mp.Event(),
                                    'start sender': mp.Event(),
                                    'running collector': mp.Event()}]
        self.laser_event_list = {'stop collector': mp.Event(),
                                 'stop sender': mp.Event(),
                                 'stop main': mp.Event(),
                                 'start collector': mp.Event(),
                                 'start sender': mp.Event(),
                                 'running collector': mp.Event()}
        self.sensor_name_0 = 'MPU_9255'
        self.sensor_name_1 = 'ISM_330'
        self.sensors_imu = [_6DOF_sender_thread._6DOF_sender_thread(start_time_in=start_time_in,
                                                                    event_list=self.sensor_event_lists[0],
                                                                    db_name_in=db_name,
                                                                    db_host_address=server_ip,
                                                                    sensor_in=self.sensor_name_0),
                            _6DOF_sender_thread._6DOF_sender_thread(start_time_in=start_time_in,
                                                                    event_list=self.sensor_event_lists[1],
                                                                    db_name_in=db_name,
                                                                    db_host_address=server_ip,
                                                                    sensor_in=self.sensor_name_1)]

        self.scanner = LASER_scanner_thread.LASER_scanner_thread(start_time_in=start_time_in,
                                                                 event_list=self.laser_event_list,
                                                                 db_name_in=db_name,
                                                                 db_host_address=server_ip,
                                                                 scanner_ip_in=scanner_ip)

    def start_threads(self):
        print('Initializing sensors and processes')
        # Start processes
        self.sensors_imu[0].start()
        self.sensors_imu[1].start()
        self.scanner.start()
        # Start_collector and senders for all processes
        self.sensor_event_lists[0]['start_sender'].set()
        self.sensor_event_lists[0]['start_collector'].set()
        self.sensor_event_lists[1]['start_sender'].set()
        self.sensor_event_lists[1]['start_collector'].set()
        self.laser_event_list['start_sender'].set()
        self.laser_event_list['start_collector'].set()

    def run_threads(self):
        # wait for scanner to lay flat
        print('Waiting for scanner to lay flat! Put the scanner down! Turn scanning off!')
        while not (self.sensors_imu[0].scanner_laying and
                   self.sensors_imu[1].scanner_laying and
                   not self.trigger.get_state()):
            while self.trigger.get_state():
                print('Turn trigger off!')
                sleep(1)
            sleep(1)
            print('Put scanner down and wait!')
        # Start sending
        self.sensors_imu[0].send_data = True
        self.sensors_imu[1].send_data = True

        print('Starting acquisition!')
        # start imu data acquisition
        while not self.main_events['end'].is_set():
            trigger_state = self.trigger.get_state()
            if trigger_state != self.scanning_command:
                self.scanning_command = trigger_state
                self.scanner.send_data = trigger_state
                if trigger_state:
                    print('Scanning started!')
                else:
                    print('Scanning paused!')
            sleep(0.05)

    def end_threads(self):
        print('Ending work')
        # stop sending laser data and end laser process
        self.scanning_command = False
        self.scanner.send_data = False
        print('Laser scanner sending finished, calling processes stop')
        self.laser_event_list['stop collector'].set()
        self.laser_event_list['stop main'].set()
        print('Waiting for laser process to end')
        self.scanner.join()

        # wait for scanner to be put down
        while not (self.sensors_imu[0].scanner_laying and
                   self.sensors_imu[1].scanner_laying):
            sleep(1)
            print('Put scanner down and wait!')

        # stop sensor data sending
        print('Stopping sending of sensor data')
        self.sensors_imu[0].send_data = False
        self.sensors_imu[1].send_data = False

        # stop sensor processes
        print('Calling end of sensor processes')
        self.sensor_event_lists[0]['stop collector'].set()
        self.sensor_event_lists[1]['stop collector'].set()
        self.sensor_event_lists[0]['stop main'].set()
        self.sensor_event_lists[1]['stop main'].set()
        print('Waiting for processes end')

        self.sensors_imu[0].join()
        self.sensors_imu[1].join()

    def run(self):
        # Threads initialization
        self.start_threads()
        # Main loop
        self.run_threads()
        # Closing and joining threads
        self.end_threads()


# Test specific code

# Test calling
if __name__ == '__main__':
    start_time = datetime.datetime.utcnow().timestamp()
    stop_event = {'end': threading.Event()}
    main_process = ThreadOrchestrator(start_time_in=start_time,
                                      server_ip='192.168.1.3',
                                      scanner_ip='192.168.1.30',
                                      db_name='scanner_tests',
                                      events_in=stop_event)

    main_process.start()

    root = tk.Tk()
    var = tk.IntVar()
    button = tk.Button(root, text="Click to end", command=lambda: var.set(1))
    button.place(relx=.5, rely=.5, anchor="c")
    button.wait_variable(var)

    stop_event['end'].set()
    main_process.join()
