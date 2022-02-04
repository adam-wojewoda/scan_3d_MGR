#import sys
#sys.path.append("SDK")
from LASER_scanner.LASER_scanner_class import LASER_scanner  # laser scanner library
#import sys  # for importing of DB

#sys.path.append("../../rp_client")
import pandas as pd  # for influx sender class
#import threading  # for parallelism
from time import sleep  # for basic pausing
import datetime  # for measurement timing (in microseconds)
import Database_sender
import multiprocessing as mp

packet_length_scan = 25
#acquisition_freq = 10 # Hz

def send_profile_process(list_in, db_host_address_in, db_in):
    list_of_points = []
    # list_in format {'time_now':time_now,'time_rel':time_rel,'line':self.scanner.get_line()}
    for line in list_in:
        time_now = line['time_now']
        time_rel = line['time_rel']
        for i in range(len(line['line'])):
            if line['line'][i]['z'] != 0.0:
                list_of_points.append([time_now - datetime.timedelta(microseconds=i), time_rel, line['line'][i]['x'],
                                       line['line'][i]['z']])
                # Microseconds added in order for server to keep all the points (datetime has microsecond resolution)
                # Without that distinction only the last point would be kept 
                # On PC side only the relative time will be used, so we have no problem here untill we read 
                # more than 10^6 points per second. Now we have 200k pts/s at max, so we're safe.
    client = Database_sender.Database_sender(host=db_host_address_in)
    df_temp = pd.DataFrame(list_of_points, columns=['time_abs', 'time_rel', 'point_X',
                                                    'points_Z'])  # .set_index(self.names[0]xy)#, index=[self.names[0]])
    df_temp.set_index('time_abs', inplace=True)
    df_temp.index = pd.to_datetime(df_temp.index, utc=True)  # , unit='us')
    client.send_measurement(df=df_temp, db_name=db_in, measurement_name='scanner_raw')


class LASER_scanner_process(mp.Process):
    def __init__(self, start_time_in, event_list, db_name_in, db_host_address, scanner_ip_in=None):
        """

        :param start_time_in: Microsecond utc time of measurement start.
        :param event_list: Thread events for control of thread work. Dict of {name:thread}
        :param db_name_in: Name od InfluxDb database to send to.
        :param db_host_address: IP address of InfluxDb server.
        :param scanner_ip_in: ip occupied by used scanner
        """

        mp.Process.__init__(self)
        self.events = event_list
        # Connect_to_scanner
        self.scanner = LASER_scanner(ip_addr_in=scanner_ip_in)
        self.start_time = start_time_in
        #self.send_data = False
        self.db_name = db_name_in
        # Create queue object
        self.q = mp.Queue()
        #self.deq = False  # Dequeueing flag (Only one so I don't use locks)
        self.internal_events = {'dequeueing': mp.Event()} # Dequeueing flag (Only one)
        # Create DB client
        self.db_client = Database_sender.Database_sender(host=db_host_address)
        self.db_address = db_host_address
        if not self.db_client.check_connection():
            raise RuntimeError('No dataframe connection')
        # Create collector thread
        self.collector = mp.Process(target=self.scanner_data_collector)
        # Create sender thread
        self.sender = mp.Process(target=self.scanner_data_sender)
        # Just to be sure they exist
        self.time_now_old = datetime.datetime.utcnow()
        self.time_rel_old = self.time_now_old.timestamp() - self.start_time
        self.time_now_new = datetime.datetime.utcnow()
        self.time_rel_new = self.time_now_new.timestamp() - self.start_time

    def __del__(self):  # As I'm using a .so lib for the scanner I want to cleanup properly
        print('LASER: Joining scanner collector process')
        self.collector.join()
        print('LASER: Joining scanner sender process')
        self.sender.join()
        print('LASER: Deleting the scanner class')
        del self.scanner

    def run(self):
        while not self.events['stop main'].is_set():
            if self.events['start collector'].is_set() and not self.events['running collector'].is_set():
                self.collector = mp.Process(target=self.scanner_data_collector)
                self.collector.start()  # Start collector thread
                self.events['start collector'].clear()  # Clear flag for later use
            if self.events['start sender'].is_set() and not self.events['running sender'].is_set():
                self.sender = mp.Process(target=self.scanner_data_sender)
                self.sender.start()  # Start sender thread
                self.events['start sender'].clear()  # Clear flag for later use
            sleep(0.05)
        self.events['stop main'].clear()  # Clear flag for later use

        self.events['stop collector'].set()  # End data collection
        sleep(0.05)
        print('LASER: run func waiting for collector to end')
        self.collector.join(timeout=2)
        if self.collector.is_alive():  # There's a bug terminating the process, but
            if self.events['running collector'].is_set():
                print('LASER: Collector still runs active, something went wrong')
            else:
                print('LASER: Collector ended its work properly but the process somehow hangs')
            print('LASER: collector forced termination was needed')
            self.collector.terminate()

        self.events['stop sender'].set()  # End data collection
        sleep(0.05)
        print('LASER: run func waiting for sender to end')
        self.sender.join(timeout=2)
        
        if self.events['running sender'].is_set():
            print('LASER: sender forced termination was needed')
            self.sender.terminate()
        print('LASER: run func sender joined')

    def scanner_data_collector(self):
        """Thread for collecting data from sensor"""
        self.events['running collector'].set()
        print('LASER: Starting collector')
        #self.time_now_old = datetime.datetime.utcnow()
        #self.time_rel_old = self.time_now_old.timestamp() - self.start_time
        #self.time_now_new = datetime.datetime.utcnow()
        #self.time_rel_new = self.time_now_new.timestamp() - self.start_time
        list_of_profiles = []
        while not self.events['stop collector'].is_set():
            #print('collector running')
            if self.events['send data'].is_set():
                # check_dump_lenght
                #if self.scanner.dump_full(): # If we've collected profiles 
                #    print('dump_full')
                #    # get profiles
                #    list_tmp = self.scanner.get_dump()
                #    # get time for new profiles
                    time_now = datetime.datetime.utcnow()
                    time_rel = self.time_now_new.timestamp() - self.start_time
                    # start recording new profiles
                #    self.scanner.dump_record_start()
                    
                    # prepare acquired profiles for queue
                #    ind = 0
                #    for profile in list_tmp:
                #        list_of_profiles.append({'time_now': self.time_now_old + datetime.timedelta(seconds=ind*(1/self.scanner.scanning_freq)), 
                #                                'time_rel': self.time_rel_old+ind*(1/self.scanner.scanning_freq), 
                #                                'line': profile})
                #        ind=ind+1
                    # update timing
                #    self.time_now_old=self.time_now_new
                #    self.time_rel_old=self.time_rel_new
                    
                #else: # check if we are collecting profiles
                #    if not self.scanner.recording: # start recording
                #        print('starting_recording')
                #        self.scanner.dump_record_start()
                #        self.time_now_old = datetime.datetime.utcnow()
                #        self.time_rel_old = self.time_now_old.timestamp() - self.start_time
                #    else: # take a quick nap
                #        if self.scanner.dump_fill_time>0.02:
                #            print('taking a nap')
                #            sleep(0.01)
                #        else:
                #            print('no nap for me')
                        
                    list_of_profiles.append({'time_now': time_now, 'time_rel': time_rel, 'line': self.scanner.get_line()})
                    print('profile acquired')
                #except:
                #    print('error getting data')
                #rest_time =time_rel + 1/acquisition_freq - (datetime.datetime.utcnow().timestamp()-self.start_time)
                #print(rest_time)
                #if rest_time>0:
                #    sleep(rest_time)
                #print('data_got')
            if not self.internal_events['dequeueing'].is_set() and len(list_of_profiles) > 2:
                #print('putting profiles to queue')
                self.q.put(list_of_profiles)
                list_of_profiles = []
                #print('profiles put')
        print('LASER: Caught stop collector event')
        # send remining points
        if len(list_of_profiles) > 1:  # give up on one line to make sure data is list of, not one dict
            self.q.put(list_of_profiles)
            list_of_profiles = []
        self.events['stop collector'].clear()  # Clear flag for later use
        print('LASER: Stopping collector')
        self.events['running collector'].clear()

    def scanner_data_sender(self):
        """Thread for sending data to server database"""
        self.events['running sender'].set()
        print('LASER: Starting sender')
        # Check if db exists
        if not self.db_client.check_connection():
            raise RuntimeError('DB no connection')
        # Check if database exists
        if not self.db_client.check_database(self.db_name):
            raise RuntimeError('No such database')

        # *******************************************************
        def rest_sender(length):
            self.internal_events['dequeueing'].set()
            list_of_lists_2 = []

            for i in range(length):
                try:
                    list_of_lists_2 = list_of_lists_2 + self.q.get(block=True)
                except Empty:
                    sleep(0.01)

            self.internal_events['dequeueing'].clear()
            p2 = mp.Process(target=send_profile_process, args=(list_of_lists_2, self.db_address, self.db_name))
            p2.start()
            print('LASER: Waiting for rest sender subprocess to end')
            p2.join()

        # *******************************************************

        list_of_lists = []
        list_of_processes = []
        p = mp.Process(target=send_profile_process, args=(list_of_lists, self.db_address, self.db_name))
        while not self.events['stop sender'].is_set() or self.events[
                'running collector'].is_set():  # Run while data is collected
            if self.q.qsize() > 0:
                try:
                    self.internal_events['dequeueing'].set()
                    list_of_lists = list_of_lists + self.q.get(block=True)
                    self.internal_events['dequeueing'].clear()
                except Empty:
                    self.internal_events['dequeueing'].clear()
                    sleep(0.01)

                if len(list_of_lists) > packet_length_scan and not p.is_alive():
                    p = mp.Process(target=send_profile_process, args=(list_of_lists, self.db_address, self.db_name))
                    p.start()
                    list_of_processes.append(p)
                    list_of_lists = []
            else:
                sleep(0.1)
        print('LASER: Cought stop sender event')
        print('LASER: Waiting for sender subprocesses to end')
        for p_t in list_of_processes:
            p_t.join()
        print('LASER: Sending rest of queue data')
        if self.q.qsize() > 0:
            rest_sender(self.q.qsize())
        self.events['stop sender'].clear()
        print('LASER: Stopping sender')
        self.events['running sender'].clear()

if __name__ == '__main__':
    """Test code:
      1. connect to selected sensor
      2. create and connect to database
      3. send sensor data for a few seconds
      """
    start_time = datetime.datetime.utcnow().timestamp()
    print(datetime.datetime.utcnow())

    thread_events = {'stop collector': mp.Event(),
                     'stop sender': mp.Event(),
                     'stop main': mp.Event(),
                     'start collector': mp.Event(),
                     'start sender': mp.Event(),
                     'running collector': mp.Event(),
                     'running sender': mp.Event(),
                     'send data': mp.Event()}
    scanner_ip = '192.168.1.30'
    # sensor = 'ISM_330'
    db_name = 'scan_sensor_test'
    db_ip_address = '192.168.1.47'
    scanner_thread = LASER_scanner_process(start_time_in=start_time,
                                          event_list=thread_events,
                                          db_name_in=db_name,
                                          db_host_address=db_ip_address,
                                          scanner_ip_in=scanner_ip)

    print('Testing connection')
    print('Server IP: ' + db_ip_address)
    if not scanner_thread.db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')
    if not scanner_thread.db_client.check_database(db_name):
        scanner_thread.db_client.create_database(db_name)  # Create database container on server
    print('Calling start sensor thread')
    scanner_thread.start()  # Start the thread
    print('Calling start sender')
    scanner_thread.send_data = True
    thread_events['start sender'].set()
    print('Calling start collector')
    thread_events['start collector'].set()
    sleep(5)
    # print('Calling stop collector')
    # thread_events['stop collector'].set()
    # sleep(1)
    # print('Calling start collector')
    # thread_events['start collector'].set()
    # sleep(1)
    print('Calling stop main')
    thread_events['stop main'].set()
