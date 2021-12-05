# from ..MPU_9255_I2C. import MPU_9255_reader
from MPU_9255_I2C.MPU_9255_reader import MPU_9255_reader
from ISM330DLC_reading_SPI.ISM330DLC_reader import ISM_330_reader


class _6DOF_sensor:
    """Class meant to take care of any 6DOF sensor in fashionable way suitable for later use"""

    def __init__(self, sensor_type: str):  # spi=True -> use SPI sensor
        self.sensor_type = sensor_type
        # in case of adding new sensors or need for data reformatting in one of them
        self.sensor = {
            'ISM_330': ISM_330_reader(),
            'MPU_9255': MPU_9255_reader(),
        }[sensor_type]
        self.get_sensor_data = {
            'ISM_330': self.get_sensor_data_ISM_330,
            'MPU_9255': self.get_sensor_data_MPU_9255,
        }[sensor_type]

    def get_sensor_data_ISM_330(self):
        return self.sensor.get_values()

    def get_sensor_data_MPU_9255(self):
        return self.sensor.get_values()


if __name__ == '__main__':

    try:
        print('Trying SPI sensor')
        sens = _6DOF_sensor(sensor_type='ISM_330')
        print('Getting data from SPI sensor:')
        print(sens.get_sensor_data())
        print('Great success!')
    except:
        print('SPI sensor failed!')
    try:
        print('Trying I2C sensor')
        sens = _6DOF_sensor(sensor_type='MPU_9255')
        print('Getting data from I2C sensor:')
        print(sens.get_sensor_data())
        print('Great success!')
    except:
        print('I2C sensor failed')
