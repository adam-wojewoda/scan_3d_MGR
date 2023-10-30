# Code for communication with InfluxDb database
# This code takes care of:
#   1. Creating measurements, databases inside InfluxDb
#   2. Checking existence of database and connection
#   3. Sending measurements to DB

from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS, PointSettings, WritePrecision

# API token
# IUIp9siSdX_4d_nVSPMNPoZKEO8m96MDSe36Gg07q0dMNGJdZiKEGmY4rMmf8-kd1Uqu6vyc6Ix_2CqIOds4Bg==
class Database_sender:
    def __init__(self, host='http://192.168.1.47:8086',
                 token='Y30JvLc8hPhBByNhSY1lGJPZQ8NN5OKhJQr471Bg0CvdG4bRKyfzps-r6SW3VyOt2a5rmRx2UDnl193D3ncfxg==',
                 org='main'):
        self.client = InfluxDBClient(url=host,
                                     token=token,
                                     debug=False, org=org)
        self.org = org
        point_settings = PointSettings()
        # self.write_precision = WritePrecision()
        # point_settings.add_default_tag("example-name", "ingest-data-frame")
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS, point_settings=point_settings)
        pass

    def check_connection(self):
        try:
            self.client.ping()
            return True
        except ConnectionError:
            return False

    def get_db_list(self):
        bucket_client = self.client.buckets_api()

        return [i.name for i in bucket_client.find_buckets().buckets]

    def check_database(self, db_name):
        self.client.ping()
        return db_name in self.get_db_list()

    # def create_database(self, db_name):
    #     if self.check_database(db_name):
    #         raise RuntimeError('DB already exists')
    #     else:
    #         self.client.create_database(db_name)
    #         self.client.create_retention_policy('main', duration='INF', replication='1', database=db_name, default=True,
    #                                             shard_duration='1h')

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

    def send_measurement(self, df, db_name, measurement_name):

        self.write_api.write(bucket=db_name, record=df, data_frame_measurement_name=measurement_name)  # ,batch_size=1000) #, time_precision='u'


# Test specific code

# Test calling
if __name__ == '__main__':
    print('Testing connection')
    db_host_address = 'http://192.168.1.47:8086'
    print('Server url: ' + db_host_address)
    db_client = Database_sender(host=db_host_address)
    if not db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')

    print('List of databases:')
    print(db_client.get_db_list())
    print(db_client.get_measurements_list('scan_sensor_test'))
    