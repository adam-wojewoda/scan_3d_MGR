""" Container for I2C sensors. For now I only have one, so the whole file is named after it """
import struct
import smbus2
import time
import matplotlib.pyplot as plt
import numpy as np

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
        tmp = self.read_multi_registers(58, 14)
        out_raw = list(struct.unpack_from("<hhhhhhh", bytearray(tmp))) #self.read_multi_registers(59, 14))))
        
        print(tmp)
        print([bin(x)[2:] for x in tmp ])
        print(out_raw)
        out_fin = [None] * 6
        for i in range(3):
            out_fin[i] = out_raw[i + 4] * self.gyro_multiplier
            out_fin[i + 3] = out_raw[i] * self.accel_multiplier
        print(out_fin)
        print(" ")
        return out_fin


#if __name__ == '__main__':

def live_plotter(x_vec_1, y1_data, y2_data, y3_data, y4_data, y5_data, y6_data,
                 line1_1, line2_1, line3_1, line4_1, line5_1, line6_1, identifier='', pause_time=0.001):
    if not line1_1:
        # this is the call to matplotlib that allows dynamic plotting
        plt.ion()
        fig = plt.figure(figsize=(10, 6))
        ax = fig.add_subplot(111)
        # create a variable for the line so we can later update it
        line1_1, = ax.plot(x_vec_1, y1_data, '-o', alpha=0.5)
        line2_1, = ax.plot(x_vec_1, y2_data, '-o', alpha=0.5)
        line3_1, = ax.plot(x_vec_1, y3_data, '-o', alpha=0.5)
        line4_1, = ax.plot(x_vec_1, y4_data, '-o', alpha=0.5)
        line5_1, = ax.plot(x_vec_1, y5_data, '-o', alpha=0.5)
        line6_1, = ax.plot(x_vec_1, y6_data, '-o', alpha=0.5)
        # update plot label/title
        plt.ylabel('Y Label')

        plt.title('Title: {}'.format(identifier))
        plt.show()

    # after the figure, axis, and line are created, we only need to update the y-data
    line1_1.set_ydata(y1_data)
    line2_1.set_ydata(y2_data)
    line3_1.set_ydata(y3_data)
    line4_1.set_ydata(y4_data)
    line5_1.set_ydata(y5_data)
    line6_1.set_ydata(y6_data)
    # adjust limits if new data goes beyond bounds
    if np.min(y1_data) <= line1_1.axes.get_ylim()[0] or np.max(y1_data) >= line1_1.axes.get_ylim()[1]:
        plt.ylim([np.min(y1_data) - np.std(y1_data), np.max(y1_data) + np.std(y1_data)])
    if np.min(y2_data) <= line2_1.axes.get_ylim()[0] or np.max(y2_data) >= line2_1.axes.get_ylim()[1]:
        plt.ylim([np.min(y2_data) - np.std(y2_data), np.max(y2_data) + np.std(y2_data)])
    if np.min(y3_data) <= line3_1.axes.get_ylim()[0] or np.max(y3_data) >= line3_1.axes.get_ylim()[1]:
        plt.ylim([np.min(y3_data) - np.std(y3_data), np.max(y3_data) + np.std(y3_data)])
    if np.min(y4_data) <= line4_1.axes.get_ylim()[0] or np.max(y4_data) >= line4_1.axes.get_ylim()[1]:
        plt.ylim([np.min(y4_data) - np.std(y4_data), np.max(y4_data) + np.std(y4_data)])
    if np.min(y5_data) <= line5_1.axes.get_ylim()[0] or np.max(y5_data) >= line5_1.axes.get_ylim()[1]:
        plt.ylim([np.min(y5_data) - np.std(y5_data), np.max(y5_data) + np.std(y5_data)])
    if np.min(y6_data) <= line6_1.axes.get_ylim()[0] or np.max(y6_data) >= line6_1.axes.get_ylim()[1]:
        plt.ylim([np.min(y6_data) - np.std(y6_data), np.max(y6_data) + np.std(y6_data)])
    # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
    plt.pause(pause_time)

    # return line so we can update it again in the next iteration
    return [line1_1, line2_1, line3_1, line4_1, line5_1, line6_1]
            
            
plt.style.use('ggplot')

size = 100
x_vec_2 = np.linspace(0, 1, size + 1)[0:-1]
y1_vec = np.random.randn(len(x_vec_2))
y2_vec = np.random.randn(len(x_vec_2))
y3_vec = np.random.randn(len(x_vec_2))
y4_vec = np.random.randn(len(x_vec_2))
y5_vec = np.random.randn(len(x_vec_2))
y6_vec = np.random.randn(len(x_vec_2))
line1_2 = []
line2_2 = []
line3_2 = []
line4_2 = []
line5_2 = []
line6_2 = []

sensor_i2c = MPU_9255_reader()

while True:
    out_fin = sensor_i2c.get_values()

    y1_vec[-1] = out_fin[0] #/ 1000
    y2_vec[-1] = out_fin[1] #/ 1000
    y3_vec[-1] = out_fin[2] #/ 1000
    y4_vec[-1] = out_fin[3] / 1000
    y5_vec[-1] = out_fin[4] / 1000
    y6_vec[-1] = out_fin[5] / 1000

    line_temp = live_plotter(x_vec_2, y1_vec, y2_vec, y3_vec, y4_vec, y5_vec, y6_vec,
                             line1_2, line2_2, line3_2, line4_2, line5_2, line6_2)
    line1_2 = line_temp[0]
    line2_2 = line_temp[1]
    line3_2 = line_temp[2]
    line4_2 = line_temp[3]
    line5_2 = line_temp[4]
    line6_2 = line_temp[5]
    y1_vec = np.append(y1_vec[1:], 0.0)
    y2_vec = np.append(y2_vec[1:], 0.0)
    y3_vec = np.append(y3_vec[1:], 0.0)
    y4_vec = np.append(y4_vec[1:], 0.0)
    y5_vec = np.append(y5_vec[1:], 0.0)
    y6_vec = np.append(y6_vec[1:], 0.0)
