""" Container for I2C sensors. For now I only have one, so the whole file is named after it """
import struct
import smbus2
import time


# def main(args):
#    return 0


class My_I2C_sensor:
    """ General I2C sensor communication class.
         For now I only have one sensor on one physical channel, so there's no need for parametrization.
         """

    def __init__(self):
        # initialize I2C bus, frequency is set by the system in boot/config.txt
        self.bus = smbus2.SMBus(1)
        self.address_dev = 0x68

    def set_register(self, address, value):
        """ Set value of single register
         :parameter address - Starting address
         :parameter value - Register input value interpreted as int, but I care only about bits"""
        # make sure we have the write command on address
        if address > 0b01111111:
            raise ValueError("Address buffer overflow-address bigger than 128")
        if value > 0b11111111:
            raise ValueError("Value bigger than 8 bit register")
        # reply = self.bus.write_byte_data(self.address_dev, address, value)
        self.bus.write_byte_data(self.address_dev, address, value)

    def read_multi_registers(self, start_address, elem_num=1):
        """ Read values from series of registers
         :parameter start_address - first address to get data from
         :parameter elem_num - number of registers to read
         :returns list of raw register data"""
        # make sure we have the write command on address
        if start_address > 0b01111111:
            raise ValueError("Address too big")
        if elem_num < 1:
            raise ValueError("Cannot read zero registers")
        reply = self.bus.read_i2c_block_data(self.address_dev, start_address, elem_num, force=True)
        # print(reply)
        return reply

    def read_single_register(self, start_address):
        """ Read value of single register
         :parameter start_address - address of register to read
         :returns single 8 bit integer value"""
        # make sure we have the write command on address
        if start_address > 0b01111111:
            raise ValueError("Address too big")
        reply = self.bus.read_byte_data(self.address_dev, start_address)
        return reply


class MPU_9255_reader(My_I2C_sensor):
    """ MPU_9255 sensor class
    The class takes care of communication and data recalculation """
    # change according to data range
    accel_multiplier = 8 / 32767 * 9.80665  # 8g
    gyro_multiplier = 1000 / 32768  # 1000 degrees per second

    def __init__(self):
        # initialize serial connection to device
        My_I2C_sensor.__init__(self)
        # self.bus.enable_pec()
        # Writing config
        # fifo and filter blocking
        self.set_register(26, 0b00000000)
        time.sleep(0.05)
        # gyro_config
        self.set_register(27, 0b00010000)
        time.sleep(0.05)
        # accel_config
        self.set_register(28, 0b00010000)
        time.sleep(0.05)
        # accel_config_2
        self.set_register(29, 0b00000000)
        time.sleep(0.05)
        # data_refreshing_config
        self.set_register(107, 0b00000000)
        time.sleep(0.05)

    def get_values(self):
        """Get gyroscope and accelerometer data
        :returns list od gyroscope and accelerometer data [gyro_X,gyro_Y,gyro_Z, acc_X, acc_Y,acc_Z] deg/s, m/s^2 """
        out_raw = list(struct.unpack_from("<hhhhhhh", bytearray(self.read_multi_registers(58, 14))))
        out_fin = [None] * 6
        for i in range(3):
            out_fin[i] = out_raw[i + 4] * self.gyro_multiplier
            out_fin[i + 3] = out_raw[i] * self.accel_multiplier
        return out_fin


if __name__ == '__main__':

    sensor_i2c = MPU_9255_reader()
    count = 0
    time_temp = time.time()
    time_start = time_temp
    time_2 = time.time()
    try:
        temp_1 = sensor_i2c.get_values()
    except TimeoutError:
            print('timeout: ' + str(time.time() - time_2))
            print('total_run: ' + str(time.time() - time_start))
            raise TimeoutError
            
    #while True:
    for j in range(10000):
        try:
            temp = sensor_i2c.get_values()
            for i in range(len(temp)):
                temp_1[i] = temp_1[i] + temp[i]
            time_2 = time.time()
            count = count + 1
            if count > 1000:
                print(1000 / (time.time() - time_temp))
                print([x/(count+1) for x in temp_1])
                count = 0
                time_temp = time.time()
                temp_1 = temp
        except TimeoutError:
            print('timeout: ' + str(time.time() - time_2))
            print('total_run: ' + str(time.time() - time_start))
            raise TimeoutError
