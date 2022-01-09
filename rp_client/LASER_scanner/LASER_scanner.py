#import sys
#sys.path.append("SDK")
from SDK.PYSDK_SMART import *

class LASER_scanner():
	def __init__(self,ip_addr_in='192.168.1.30'):
		self.scanner=None
		
		# Initialize sdk for the scanner
		sdk_init()
		
		# Search for scanners in available network 
		list_scanners=search()
		
		# Check if the scanner I want is in the network 
		# Easier than calling single scanner in this SDK
		for scanner in list_scanners: 
			if get_info(scanner, kSERVICE)['user_network_ip']==ip_addr_in:
				self.scanner = scanner
		if self.scanner == None:
			raise RuntimeError('Unable to find scanner in network')
			
		self.is_connected = connect(self.scanner)
		if (not self.is_connected):
			raise RuntimeError("Failed to connect to scanner!")
		self.zero_points=True
		self.realtime=True
		
	def __del__(self):
		# Disconnect the scanner
		disconnect(self.scanner)
		# Cleanup resources allocated with scanner sdk
		sdk_cleanup()
		
	def get_line(self):
		# Get profile
		profile = get_profile2D(self.scanner,self.zero_points,self.realtime,kSERVICE)
		# Check if the output data is what I want
		if (self.is_connected and profile is not None):
			profile_data_type=profile['header']['data_type']
			if profile_data_type == PROFILE_DATA_TYPES.PROFILE:
				profile_out=[]
				for point in profile['points']: 
					profile_out.append({'x':point.x,'z':point.z})
					# Copying of data to get rid of SDK mutable object as fast as possible 
				return(profile_out.copy())
			else:
				raise RuntimeError("Scanner output datatype is not profile! Change output type")
		else:
			raise RuntimeError("Profile not recieved!")		
		
if __name__ == '__main__':
	"""Test code:
	  1. connect to scanner
	  2. try getting a profile
	  3. end connection
	  
	  Then do the same but for some time in order to get timing
	  """
	
	print("Connect")
	scanner=LASER_scanner('192.168.1.30')
	print("Get_line")
	prof = scanner.get_line()
	print("Try printing")
	for point in prof:
		if point['z'] != 0.0:
			print('Z: ' + str(point['z']) + '	X: ' + str(point['x']))
	print("Finished!")
	
	import time  # for basic pausing (reliving CPU)
	
	print("Checking timing")
	if True:
		#print("Connect")
		#scanner=LASER_scanner()
		print("Get lines for more or less 5s")
		for i in range(5):
			t1 = time.time()
			cnt=0
			while t1+1>time.time():
				try:
					prof = scanner.get_line()
					time.sleep(0.001)
					cnt=cnt+1
				except:
					print("Error cought")
			print(cnt)
		
