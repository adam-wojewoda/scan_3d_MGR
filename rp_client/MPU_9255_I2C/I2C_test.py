if __name__ == '__main__':
    import smbus2

    bus = smbus2.SMBus(1)
    address = 0x68

    for i in range(128):
        print(str(i) + ': ' + str(bus.read_byte_data(address, i)))
