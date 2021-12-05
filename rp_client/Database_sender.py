# Code for communication with InfluxDb database
# This code takes care of:
#   1. Creating measurements, databases inside of InfluxDb
#   2. Checking existence of database and connection
#   3. Sending measurements to DB

from influxdb import DataFrameClient


class Database_sender:
    def __init__(self, host, port=8086, user='root', password='root'):
        self.client = DataFrameClient(host, port, user, password)
        pass

    def check_connection(self):
        try:
            self.client.ping()
            return True
        except ConnectionError:
            return False

    def check_database(self, db_name):
        self.client.ping()
        return {'name':db_name} in self.client.get_list_database()

    def create_database(self, db_name):
        if self.check_database(db_name):
            raise RuntimeError('DB already exists')
        else:
            self.client.create_database(db_name)
            self.client.create_retention_policy('main', duration='INF', replication='1', database=db_name, default=True,
                                                shard_duration='1h')

    def check_measurement(self, measurement_name, db_name):
        if not self.check_database(db_name):
            raise RuntimeError('DB does not exist')
        else:
            return measurement_name in self.client.get_list_measurements()

    def send_measurement(self, df, db_name, measurement_name):
        self.client.write_points(df, measurement_name, protocol='line', database=db_name, time_precision='u')

    def get_whole_measurement(self, db_name, measurement_name):
        return self.client.query('select * from %(measurement)s' % {"measurement": measurement_name}, epoch='u',
                                 database=db_name).get_points()


# Test specific code

# Test calling
if __name__ == '__main__':
    print('Testing connection')
    db_host_address='192.168.10.15'
    print('Server IP: ' + db_host_address)
    db_client = Database_sender(host=db_host_address)
    if not db_client.check_connection():
        raise RuntimeError('No dataframe connection')
    else:
        print('Connection ok')
        
    print('List of databases:')
    print(db_client.client.get_list_database())
    #print('Lack of testing code')
    
    
