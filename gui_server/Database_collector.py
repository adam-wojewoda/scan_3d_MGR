# Code for communication with InfluxDb database
# This code takes care of:
#   1. Creating measurements, databases inside of InfluxDb
#   2. Checking existence of database and connection
#   3. Sending measurements to DB

from influxdb import DataFrameClient



class Database_collector:
    def __init__(self, host, port=8086, user='root', password='root'):
        self.client = DataFrameClient(host, port, user, password)
        pass

    def check_connection(self):
        try:
            self.client.ping()
            return True
        except ConnectionError:
            return False

    def create_database(self, db_name):
        if self.check_database(db_name):
            raise RuntimeError('DB already exists')
        else:
            self.client.create_database(db_name)
            self.client.create_retention_policy('main', duration='INF', replication='1', database=db_name, default=True,
                                                shard_duration='1h')

    def get_db_list(self):
        return [i['name'] for i in self.client.get_list_database()]

    def check_database(self, db_name):
        self.client.ping()
        return db_name in self.get_db_list()

    def get_measurements_list(self, db_name):
        if not self.check_database(db_name):
            raise RuntimeError('DB does not exist')
        self.client.switch_database(db_name)
        return [i['name'] for i in self.client.get_list_measurements()]

    def check_measurement(self, measurement_name, db_name):
        return measurement_name in self.get_measurements_list(db_name)

    def send_measurement(self, df, db_name, measurement_name):
        self.client.write_points(df, measurement_name, protocol='line', database=db_name,
                                 time_precision='u')  # ,batch_size=1000) #, time_precision='u'

    def get_measurement_data(self, db_name, measurement_name, timestamp_start=None, timestamp_end=None):
        if timestamp_start is None and timestamp_end is None:
            return self.client.query(
                'select mean(*) from %(measurement)s GROUP BY time(1ms) fill(linear)' % {
                    "measurement": measurement_name}, epoch='u',
                database=db_name)[measurement_name]
        elif timestamp_end > timestamp_start:  # This should raise an error when one is none
            return self.client.query(
                'select mean(*) from %(measurement)s WHERE time >= %(t_s)sms and time <= %(t_e)sms GROUP BY time(1ms) '
                'fill(linear) '
                % {"measurement": measurement_name, 't_s':timestamp_start, 't_e': timestamp_end}, epoch='u',
                database=db_name)[measurement_name]
        else:
            raise ValueError('End earlier than start!')


# Test specific code

# Test calling
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import pandas as pd
    print('Testing connection')
    db_host_address = 'localhost'
    print('Server IP: ' + db_host_address)
    db_client = Database_collector(host=db_host_address)
    if not db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')

    print('List of databases:')
    print(db_client.get_db_list())

    print('List of measurements:')
    print(db_client.get_measurements_list('scan_sensor_test'))
    data = db_client.get_measurement_data('scan_sensor_test', 'MPU_9255_raw', 1643975248642,1643975333534).dropna()
    with pd.option_context('display.max_rows', None, 'display.max_columns', None,
                           'display.precision', 3,
                           ):
        print(data.describe())

    plt.style.use('dark_background')

    plt.plot(data['mean_time_rel'], data['mean_acc_Y'])
    plt.show()
    print('Lack of testing code')
