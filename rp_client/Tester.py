import datetime
from _6DOF_sender_process import _6DOF_sender_process
from LASER_scanner.LASER_scanner_process import LASER_scanner_process
from LASER_scanner import Measurement_button
import multiprocessing as mp
#import threading
from ProcessOrchestrator import get_sensor_events, get_scanner_events
from time import sleep


if __name__ == '__main__':
    """Test code sensor 0:
      1. connect to selected sensor
      2. create and connect to database
      3. send sensor data for a few seconds
      """
    if False:
        start_time = datetime.datetime.utcnow().timestamp()
        thread_events = get_sensor_events()
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
        for i in range(1000):
            sleep(0.1)
            print('running as hell')
        print('Calling stop collector')
        thread_events['stop collector'].set()
        # sleep(1)
        # print('Calling start collector')
        # thread_events['start collector'].set()
        # sleep(1)
        print('Calling stop main')
        thread_events['stop main'].set()

    """Test code sensor 1:
      1. connect to selected sensor
      2. create and connect to database
      3. send sensor data for a few seconds
      """
        
    if False:
        start_time = datetime.datetime.utcnow().timestamp()
        thread_events = get_sensor_events()
        sensor = 'MPU_9255'
        #sensor = 'ISM_330'
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
            sleep(10)
        print('Calling stop collector')
        thread_events['stop collector'].set()
        # sleep(1)
        # print('Calling start collector')
        # thread_events['start collector'].set()
        # sleep(1)
        print('Calling stop main')
        thread_events['stop main'].set()

    if True:
        """Test code LASER:
        1. connect to selected sensor
        2. create and connect to database
        3. send sensor data for a few seconds
        """
        start_time = datetime.datetime.utcnow().timestamp()
        print(datetime.datetime.utcnow())

        thread_events = get_scanner_events()
        scanner_ip = '192.168.1.30'
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
        thread_events['send data'].set()
        thread_events['start sender'].set()
        print('Calling start collector')
        thread_events['start collector'].set()
        for i in range(1000):
            sleep(0.1)
            print('running as hell')
        # print('Calling stop collector')
        # thread_events['stop collector'].set()
        # sleep(1)
        # print('Calling start collector')
        # thread_events['start collector'].set()
        # sleep(1)
        print('Calling stop main')
        thread_events['stop main'].set()
