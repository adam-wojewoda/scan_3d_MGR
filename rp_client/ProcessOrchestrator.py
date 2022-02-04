# Code for main loop of data acquisition from scanner
# This code takes care of:
#   1. Calling all scanner side processes
#   2. Sending the data to database
#   3. Close the program after finishing

import datetime
from _6DOF_sender_process import _6DOF_sender_process
from LASER_scanner.LASER_scanner_process import LASER_scanner_process
from LASER_scanner import Measurement_button
import multiprocessing as mp
#import threading
from time import sleep
import tkinter as tk

def get_sensor_events():
    return {'stop collector': mp.Event(),
            'stop sender': mp.Event(),
            'stop main': mp.Event(),
            'start collector': mp.Event(),
            'start sender': mp.Event(),
            'running collector': mp.Event(),
            'running sender': mp.Event(),
            'send data': mp.Event(),
            'sensor laying':mp.Event()}
def get_scanner_events():
    return {'stop collector': mp.Event(),
            'stop sender': mp.Event(),
             'stop main': mp.Event(),
             'start collector': mp.Event(),
             'start sender': mp.Event(),
             'running collector': mp.Event(),
             'running sender': mp.Event(),
             'send data': mp.Event()}
    

class ProcessOrchestrator(mp.Process):
    def __init__(self, start_time_in, server_ip, scanner_ip, db_name, events_in):
        """Create all the threads, prepare for some work"""
        mp.Process.__init__(self)
        self.trigger = Measurement_button.Measurement_button(11)
        self.trigger_state = False
        self.main_events = events_in
        self.sensor_event_lists = [get_sensor_events(),
                                   get_sensor_events()]
        self.laser_event_list = get_scanner_events()
        self.sensor_name_0 = 'MPU_9255'
        self.sensor_name_1 = 'ISM_330'
        self.sensors_imu = [_6DOF_sender_process(start_time_in=start_time_in,
                                                 event_list=self.sensor_event_lists[0],
                                                 db_name_in=db_name,
                                                 db_host_address=server_ip,
                                                 sensor_in=self.sensor_name_0),
                            _6DOF_sender_process(start_time_in=start_time_in,
                                                 event_list=self.sensor_event_lists[1],
                                                 db_name_in=db_name,
                                                 db_host_address=server_ip,
                                                 sensor_in=self.sensor_name_1)]

        self.scanner = LASER_scanner_process(start_time_in=start_time_in,
                                             event_list=self.laser_event_list,
                                             db_name_in=db_name,
                                             db_host_address=server_ip,
                                             scanner_ip_in=scanner_ip)
        print('MAIN: Initializing sensors and processes')
        # Start processes
        self.sensors_imu[0].start()
        print('MAIN: IMU_0 started')
        self.sensors_imu[1].start()
        print('MAIN: IMU_1 started')
        self.scanner.start()
        print('MAIN: scanner started')                                     

    def start_processes(self):
        print('MAIN: Initializing sensors and processes')
        # Start processes
        self.sensors_imu[0].start()
        print('MAIN: IMU_0 started')
        self.sensors_imu[1].start()
        print('MAIN: IMU_1 started')
        self.scanner.start()
        print('MAIN: scanner started')
        # Start_collector and senders for all processes
        self.sensor_event_lists[0]['start sender'].set()
        print('MAIN: IMU_0 sender started')
        self.sensor_event_lists[0]['start collector'].set()
        print('MAIN: IMU_0 collector started')
        self.sensor_event_lists[1]['start sender'].set()
        print('MAIN: IMU_1 sender started')
        self.sensor_event_lists[1]['start collector'].set()
        print('MAIN: IMU_1 collector started')
        self.laser_event_list['start sender'].set()
        print('MAIN: LASER sender started')
        self.laser_event_list['start collector'].set()
        print('MAIN: LASER collector started')
        print('MAIN: Start flags set')

    def run_processes(self):
        # wait for scanner to lay flat
        print('MAIN: Waiting for scanner to lay flat! Put the scanner down! Turn scanning off!')
        
        print(self.sensor_event_lists[0]['sensor laying'].is_set())
        print(self.sensor_event_lists[1]['sensor laying'].is_set())
        print(self.trigger.get_state())
        
        while not (self.sensor_event_lists[0]['sensor laying'].is_set() and
                   self.sensor_event_lists[1]['sensor laying'].is_set() and
                   not self.trigger.get_state()):
            #print ('Inside loop')
            if self.trigger.get_state():
                print('Turn trigger off!')
            sleep(2)
            if not self.sensor_event_lists[0]['sensor laying'].is_set():
                print('Waiting for sensor 0.')# g_tot: ', self.sensors_imu[0].movement_sensor.get_g())
            if not self.sensor_event_lists[1]['sensor laying'].is_set():
                print('Waiting for sensor 1.')# g_tot: ', self.sensors_imu[1].movement_sensor.get_g())
            
        # Start sending
        self.sensor_event_lists[0]['send data'].set()
        self.sensor_event_lists[1]['send data'].set()

        print('MAIN: Starting acquisition!')
        # start imu data acquisition
        while not self.main_events['end'].is_set():
            trigger_state = self.trigger.get_state()
            if trigger_state != self.scanning_command:
                self.scanning_command = trigger_state
                self.scanner.send_data = trigger_state
                if trigger_state:
                    print('MAIN: Scanning started!')
                else:
                    print('MAIN: Scanning paused!')
            sleep(0.05)

    def end_processes(self):
        print('MAIN: Ending work')
        # stop sending laser data and end laser process
        self.laser_event_list['send_data'].clear()
        print('MAIN: Laser scanner sending finished, calling processes stop')
        self.laser_event_list['stop collector'].set()
        self.laser_event_list['stop main'].set()
        print('MAIN: Waiting for laser process to end')
        self.scanner.join()

        # wait for scanner to be put down
        while not (self.sensors_imu[0].scanner_laying and
                   self.sensors_imu[1].scanner_laying):
            sleep(1)
            print('MAIN: Put scanner down and wait!')

        # stop sensor data sending
        print('MAIN: Stopping sending of sensor data')
        self.sensor_event_lists[0]['send data'].clear()
        self.sensor_event_lists[1]['send data'].clear()

        # stop sensor processes
        print('MAIN: Calling end of sensor processes')
        self.sensor_event_lists[0]['stop collector'].set()
        self.sensor_event_lists[1]['stop collector'].set()
        self.sensor_event_lists[0]['stop main'].set()
        self.sensor_event_lists[1]['stop main'].set()
        print('MAIN: Waiting for processes end')

        self.sensors_imu[0].join()
        self.sensors_imu[1].join()

    def run(self):
        # Threads initialization
        #self.start_processes()
        # Main loop
        #self.run_processes()
        # Closing and joining threads
        #self.end_processes()
            #def start_processes(self):
        #print('Initializing sensors and processes')
        # Start processes
        #self.sensors_imu[0].start()
        #print('IMU_0 started')
        #self.sensors_imu[1].start()
        #print('IMU_1 started')
        #self.scanner.start()
        #print('scanner started')
        # Start_collector and senders for all processes
        self.sensor_event_lists[0]['start sender'].set()
        print('MAIN: IMU_0 sender started')
        self.sensor_event_lists[0]['start collector'].set()
        print('MAIN: IMU_0 collector started')
        self.sensor_event_lists[1]['start sender'].set()
        print('MAIN: IMU_1 sender started')
        self.sensor_event_lists[1]['start collector'].set()
        print('MAIN: IMU_1 collector started')
        self.laser_event_list['start sender'].set()
        print('MAIN: LASER sender started')
        self.laser_event_list['start collector'].set()
        print('MAIN: LASER collector started')
        print('MAIN: Start flags set')

        #def run_processes(self):
        # wait for scanner to lay flat
        print('MAIN: Waiting for scanner to lay flat! Put the scanner down! Turn scanning off!')
        
        print(self.sensor_event_lists[0]['sensor laying'].is_set())
        print(self.sensor_event_lists[1]['sensor laying'].is_set())
        print(self.trigger.get_state())
        
        while not (self.sensor_event_lists[0]['sensor laying'].is_set() and
                   self.sensor_event_lists[1]['sensor laying'].is_set() and
                   not self.trigger.get_state()):
            #print ('Inside loop')
            if self.trigger.get_state():
                print('MAIN: Turn trigger off!')
            sleep(2)
            if not self.sensor_event_lists[0]['sensor laying'].is_set():
                print('MAIN: Waiting for sensor 0.')# g_tot: ', self.sensors_imu[0].movement_sensor.get_g())
            if not self.sensor_event_lists[1]['sensor laying'].is_set():
                print('MAIN: Waiting for sensor 1.')# g_tot: ', self.sensors_imu[1].movement_sensor.get_g())
            
        # Start sending
        self.sensor_event_lists[0]['send data'].set()
        self.sensor_event_lists[1]['send data'].set()

        print('MAIN: Starting acquisition!')
        # start imu data acquisition
        while not self.main_events['end'].is_set():
            #print('end_check')
            trigger_state_temp = self.trigger.get_state()
            if self.trigger_state!=trigger_state_temp:
                #print(trigger_state)
                if self.trigger_state:
                    self.laser_event_list['send data'].set()
                    print('MAIN: Scanning started!')
                else:
                    self.laser_event_list['send data'].clear()
                    print('MAIN: Scanning paused!')
            self.trigger_state = trigger_state_temp
            sleep(0.05)

        #def end_processes(self):
        print('MAIN: Ending work')
        # stop sending laser data and end laser process
        self.laser_event_list['send data'].clear()
        print('MAIN: Laser scanner sending finished, calling processes stop')
        self.laser_event_list['stop collector'].set()
        self.laser_event_list['stop main'].set()
        sleep(0.2)
        print('MAIN: Waiting for laser process to end')
        #self.scanner.join(timeout=0.3)
        if self.laser_event_list['running sender'].is_set():
            self.scanner.terminate()
            print('MAIN: LASER process termination was needed')
            
        print('MAIN: LASER joined!')
        # wait for scanner to be put down
        while not (self.sensor_event_lists[0]['sensor laying'].is_set() and
                   self.sensor_event_lists[1]['sensor laying'].is_set()):
            sleep(1)
            print('MAIN: Put scanner down and wait!')

        # stop sensor data sending
        print('MAIN: Stopping sending of sensor data')
        self.sensor_event_lists[0]['send data'].clear()
        self.sensor_event_lists[1]['send data'].clear()

        # stop sensor processes
        print('MAIN: Calling end of sensor processes')
        self.sensor_event_lists[0]['stop collector'].set()
        self.sensor_event_lists[1]['stop collector'].set()
        self.sensor_event_lists[0]['stop main'].set()
        self.sensor_event_lists[1]['stop main'].set()
        print('MAIN: Waiting for processes end')

        self.sensors_imu[0].join()
        self.sensors_imu[1].join()


# Test specific code

# Test calling
if __name__ == '__main__':
    start_time = datetime.datetime.utcnow().timestamp()
    stop_event = {'end': mp.Event()}
    main_process = ProcessOrchestrator(start_time_in=start_time,
                                      server_ip='192.168.1.47',
                                      scanner_ip='192.168.1.30',
                                      db_name='scan_sensor_test',
                                      events_in=stop_event)

    main_process.start()

    root = tk.Tk()
    var = tk.IntVar()
    button = tk.Button(root, text="Click to end", command=lambda: var.set(1))
    button.place(relx=.5, rely=.5, anchor="c")
    button.wait_variable(var)

    stop_event['end'].set()
    main_process.join()
