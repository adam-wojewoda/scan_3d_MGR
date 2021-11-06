import struct
import smbus2
import time


# def main(args):
#    return 0


class My_I2C_sensor:
    def __init__(self):
        # initialize I2C bus, frequency is set by the system in boot/config.txt
        self.bus = smbus2.SMBus(1)
        self.address_dev = 0x68

    def set_register(self, address, value):

        # make sure we have the write command on address
        if address > 0b01111111:
            raise ValueError("Address buffer overflow-address bigger than 128")
        if value > 0b11111111:
            raise ValueError("Value bigger than 8 bit register")
        # reply = self.bus.write_byte_data(self.address_dev, address, value)
        self.bus.write_byte_data(self.address_dev, address, value)

    def read_multi_registers(self, start_address, elem_num=1):
        # make sure we have the write command on address
        if start_address > 0b01111111:
            raise ValueError("Address too big")
        if elem_num < 1:
            raise ValueError("Cannot read zero registers")
        reply = self.bus.read_i2c_block_data(self.address_dev, start_address, elem_num, force=True)
        # print(reply)
        return reply

    def read_single_register(self, start_address):
        # make sure we have the write command on address
        if start_address > 0b01111111:
            raise ValueError("Address too big")
        reply = self.bus.read_byte_data(self.address_dev, start_address)
        return reply


class MPU_9255_reader(My_I2C_sensor):
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
        out_raw = list(struct.unpack_from("<hhhhhh", bytearray(self.read_multi_registers(59, 12))))
        out_fin = [None] * 6
        for i in range(3):
            out_fin[i + 3] = out_raw[i + 3] * self.gyro_multiplier
            out_fin[i] = out_raw[i] * self.accel_multiplier
        return out_fin


if __name__ == '__main__':

    sensor_i2c = MPU_9255_reader()
    count = 0
    time_temp = time.time()
    time_start = time_temp
    time_2 = time.time()
    while True:
        try:
            temp = sensor_i2c.get_values()
            time_2 = time.time()
            count = count + 1
            if count > 1000:
                count = 0
                print(1000 / (time.time() - time_temp))
                # print(temp)
                time_temp = time.time()
        except TimeoutError:
            print('timeout: ' + str(time.time() - time_2))
            print('total_run: ' + str(time.time() - time_start))
            raise TimeoutError
