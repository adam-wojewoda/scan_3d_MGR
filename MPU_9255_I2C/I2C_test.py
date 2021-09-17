import smbus2
import time
bus = smbus2.SMBus(1)
address = 0x68

#bear = bus.read_byte_data(address, 1)



for i in range(128):
	
	#bear255 = bearing255()      #this returns the value as a byte between 0 and 255. 
	print (str(i)+': ' +str(bus.read_byte_data(address, i)))
	#time.sleep(1)
#while True:
#	try:
#	    print(bus.read_i2c_block_data(address, 59, 12))
#	except TimeoutError:
#		print('timeout')
#		time.sleep(0.001)
