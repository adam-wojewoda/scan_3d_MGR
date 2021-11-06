import time
import spidev
import struct
import matplotlib.pyplot as plt
import numpy as np

spi_ch = 0

# Enable SPI
spi = spidev.SpiDev(0, spi_ch)
spi.max_speed_hz = 5000000
accel_multiplier = 8 / 32767 * 9.80665  # 8g
gyro_multiplier = 1000 / 32768  # 1000 degrees per second


def set_register(address, value):
    # make sure we have the write command on address
    if address > 0b01111111:
        raise ValueError("Not a write command address")
    msg = address
    if value > 0b11111111:
        raise ValueError("Value bigger than register")
    msg = [msg, value]
    # reply = spi.xfer2(msg)
    spi.xfer2(msg)


def read_multi_registers(start_address, elem_num=1):
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
    reply = spi.xfer2(msg)
    return reply[1:]


def read_single_register(start_address):
    # make sure we have the write command on address
    if start_address > 0b11111111:
        raise ValueError("Address too big")
    if start_address < 0b10000000:  # Write read_command_bit
        msg = start_address + 0b10000000
    else:
        msg = start_address
    msg = [msg, 0]
    reply = spi.xfer2(msg)
    return reply[1]


def reg_series_to_val(in_list):
    out_raw = list(struct.unpack_from("<hhhhhh", bytearray(in_list)))
    out_f = [0.0] * 6
    for i in range(3):
        out_f[i] = out_raw[i] * gyro_multiplier
        out_f[i + 3] = out_raw[i + 3] * accel_multiplier  # *multiplier

    print(out_f)
    return out_f


def live_plotter(x_vec_1, y1_data, y2_data, y3_data, y4_data, y5_data, y6_data,
                 line1_1, line2_1, line3_1, line4_1, line5_1, line6_1, identifier='', pause_time=0.1):
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

# Writing config
set_register(0x10, 0b10101100)
time.sleep(0.1)
set_register(0x11, 0b10101000)
time.sleep(0.1)
set_register(0x12, 0b01000100)
time.sleep(0.1)

while True:
    out_now = read_multi_registers(0x22, 12)
    out_fin = reg_series_to_val(out_now)

    y1_vec[-1] = out_fin[0] / 1000
    y2_vec[-1] = out_fin[1] / 1000
    y3_vec[-1] = out_fin[2] / 1000
    y4_vec[-1] = out_fin[3]
    y5_vec[-1] = out_fin[4]
    y6_vec[-1] = out_fin[5]

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
