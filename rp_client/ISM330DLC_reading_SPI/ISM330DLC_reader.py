""" Container for SPI sensors. For now I only have one, so the whole file is named after it """

import time
import spidev
import struct


# def main(args):
#    return 0


class My_SPI_sensor:
    """ General SPI sensor communication class.
     For now I only have one sensor on one physical channel, so there's no need for parametrization.
     """

    def __init__(self):
        spi_ch = 0

        # Enable SPI
        self.spi = spidev.SpiDev(0, spi_ch)
        self.spi.max_speed_hz = 5000000

    def set_register(self, address, value):
        """ Set value of single register
         :parameter address - Starting address
         :parameter value - Register input value interpreted as int, but I care only about bits
         """

        # make sure we have the write command on address
        if address > 0b01111111:
            raise ValueError("Not a write command address")
        msg = address
        if value > 0b11111111:
            raise ValueError("Value bigger than register")
        msg = [msg, value]
        # reply = self.spi.xfer2(msg)
        self.spi.xfer2(msg)

    def read_multi_registers(self, start_address, elem_num=1):
        """ Read values from series of registers
         :parameter start_address - first address to get data from
         :parameter elem_num - number of registers to read
         :returns list of raw register data"""
        # make sure we have the write command on address
        if start_address > 0b11111111:
            raise ValueError("Address too big")
        if start_address < 0b10000000:  # Write read_command_bit
            msg = start_address + 0b10000000
        else:
            msg = start_address
        if elem_num < 1:
            raise ValueError("Cannot read zero registers")
        msg = [msg]
        for i in range(elem_num):
            msg.append(0)
        reply = self.spi.xfer2(msg)
        return reply[1:]

    def read_single_register(self, start_address):
        """ Read value of single register
         :parameter start_address - address of register to read
         :returns single 8 bit integer value"""
        # make sure we have the write command on address
        if start_address > 0b11111111:
            raise ValueError("Address too big")
        if start_address < 0b10000000:  # Write read_command_bit
            msg = start_address + 0b10000000
        else:
            msg = start_address
        msg = [msg, 0]
        reply = self.spi.xfer2(msg)
        return reply[1]


class ISM_330_reader(My_SPI_sensor):
    """ ISM_330 sensor class
    The class takes care of communication and data recalculation """
    # change according to data range
    accel_multiplier = 8 / 32767 * 9.80665  # 8g
    gyro_multiplier = 1000 / 32768  # 1000 degrees per second

    def __init__(self):
        # initialize serial connection to device
        My_SPI_sensor.__init__(self)
        # Writing config
        # accel_config
        self.set_register(0x10, 0b10101100)
        time.sleep(0.05)
        # gyro_config
        self.set_register(0x11, 0b10101000)
        time.sleep(0.05)
        # data_refreshing_config
        self.set_register(0x12, 0b01000100)
        time.sleep(0.05)

    def get_values(self):
        """Get gyroscope and accelerometer data
         :returns list od gyroscope and accelerometer data [gyro_X,gyro_Y,gyro_Z, acc_X, acc_Y,acc_Z] deg/s, m/s^2 """
        out_raw = list(struct.unpack_from("<hhhhhh", bytearray(self.read_multi_registers(0x22, 12))))
        out_fin = [None] * 6
        for i in range(3):
            out_fin[i] = out_raw[i] * self.gyro_multiplier
            out_fin[i + 3] = out_raw[i + 3] * self.accel_multiplier
        return out_fin


if __name__ == '__main__':

    try:
        sensor_spi = ISM_330_reader()
        count = 0
        time_temp = time.time()
        while True:
            temp = sensor_spi.get_values()
            count = count + 1
            if count > 10000:
                count = 0
                print(10000 / (time.time() - time_temp))
                print(temp)
                time_temp = time.time()

    finally:
        exit(0)
