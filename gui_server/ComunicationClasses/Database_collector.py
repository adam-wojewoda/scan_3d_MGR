# Code for communication with InfluxDb database
# This code takes care of:
#   1. Creating measurements, databases inside InfluxDb
#   2. Checking existence of database and connection
#   3. Sending measurements to DB

from influxdb_client import InfluxDBClient


class Database_collector:
    def __init__(self, url='http://localhost:8086',
                 token='HaLc0ouWlBIpHhP4WPtiOJQI1tDoE9X3u01DqWywai7uLlyQfZLUzB1z_tDrUa1UkDQEiP23-as9l_h45djfkg==',
                 org='main'):
        self.client = InfluxDBClient(url=url,
                                     token=token,
                                     debug=False, org=org)
        self.org = org
        pass

    def check_connection(self):
        try:
            self.client.ping()
            return True
        except ConnectionError:
            return False

    # def create_database(self, db_name):
    #     if self.check_database(db_name):
    #         raise RuntimeError('DB already exists')
    #     else:
    #         self.client.create_database(db_name)
    #         self.client.create_retention_policy('main', duration='INF', replication='1', database=db_name, default=True,
    #                                             shard_duration='1h')

    def get_db_list(self):
        bucket_client = self.client.buckets_api()

        return [i.name for i in bucket_client.find_buckets().buckets]

    def check_database(self, db_name):
        self.client.ping()
        return db_name in self.get_db_list()

    def get_measurements_list(self, db_name):
        query_client = self.client.query_api()
        if db_name not in self.get_db_list():
            raise RuntimeError('DB does not exist')
        query = f"""
        import \"influxdata/influxdb/schema\"

        schema.measurements(bucket: \"{db_name}\")
        """
        tables = query_client.query(query=query, org=self.org)

        return [row.values["_value"] for table in tables for row in table]

    # def check_measurement(self, measurement_name, db_name):
    #     return measurement_name in self.get_measurements_list(db_name)
    #
    #
    # def send_measurement(self, df, db_name, measurement_name):
    #     self.client.write_points(df, measurement_name, protocol='line', database=db_name,
    #                              time_precision='u')  # ,batch_size=1000) #, time_precision='u'

    def get_measurement_data(self, db_name, measurement_name, timestamp_start=None, timestamp_end=None):
        query_client = self.client.query_api()
        if timestamp_start is None and timestamp_end is None:
            raise ValueError('No time given!')
        elif timestamp_end > timestamp_start:  # This should raise an error when one is none
            query_text = 'import "interpolate" ' \
                         'from(bucket: "{}")' \
                         '  |> range(start: {}, stop: {})' \
                         '  |> filter(fn: (r) => r["_measurement"] == "{}")' \
                         '  |> keep(columns: ["_time", "_field", "_value"])' \
                         '  |> aggregateWindow(every: 1ms, fn: mean, createEmpty: false)' \
                         '  |> interpolate.linear(every: 1ms)' \
                         '  |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")'.format(db_name,
                                                                                                             timestamp_start,
                                                                                                             timestamp_end,
                                                                                                             measurement_name)
            # print(query_text)
            df = query_client.query_data_frame(query_text).drop(['result', 'table', '_start', '_stop'], axis=1)
            df.set_index('_time', inplace=True)
            return df
        else:
            raise ValueError('End earlier than start!')

    def get_scanner_data(self, db_name, timestamp_start=None, timestamp_end=None):
        query_client = self.client.query_api()
        if timestamp_start is None and timestamp_end is None:
            raise ValueError('No time given!')
        elif timestamp_end > timestamp_start:  # This should raise an error when one is none
            query_text = 'from(bucket: "{}")' \
                         '  |> range(start: {}, stop: {})' \
                         '  |> filter(fn: (r) => r["_measurement"] == "scanner_raw")' \
                         '  |> keep(columns: ["_time", "_field", "_value"])' \
                         '  |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")' \
                         ''.format(db_name, timestamp_start, timestamp_end)
            # print(query_text)
            df = query_client.query_data_frame(query_text).drop(['result', 'table'], axis=1)
            df.set_index('_time', inplace=True)
            return df
            # return self.client.query(
            #     'select * from %(measurement)s WHERE time >= %(t_s)sms and time <= %(t_e)sms '
            #     % {"measurement": 'scanner_raw', 't_s': timestamp_start, 't_e': timestamp_end}, epoch='ns',
            #     database=db_name)['scanner_raw']
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
    db_client = Database_collector()
    if not db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')

    print('List of databases:')
    print(db_client.get_db_list())

    print('List of measurements:')
    print(db_client.get_measurements_list('scan_sensor_test'))
    data2 = db_client.get_scanner_data('scan_sensor_test', 1652530355, 1652530381).dropna()
    data = db_client.get_measurement_data('scan_sensor_test', 'MPU_9255_raw', 1652530355, 1652530381).dropna()

    with pd.option_context('display.max_rows', None, 'display.max_columns', None,
                           'display.precision', 3,
                           ):
        print(data2.head())

    with pd.option_context('display.max_rows', None, 'display.max_columns', None,
                           'display.precision', 3,
                           ):
        print(data.head())

    data.set_index('time_rel', inplace=True)
    with pd.option_context('display.max_rows', None, 'display.max_columns', None,
                           'display.precision', 3,
                           ):
        print(data.head())
    # data.drop('')
    plt.style.use('dark_background')
    figure, axis = plt.subplots(1, 2)
    axis[0].plot(data.index, data['acc_X'])
    axis[0].plot(data.index, data['acc_Y'])
    axis[0].plot(data.index, data['acc_Z'])

    axis[1].plot(data.index, data['gyro_X'])
    axis[1].plot(data.index, data['gyro_Y'])
    axis[1].plot(data.index, data['gyro_Z'])

    plt.show()
