#import sys
#sys.path.append("SDK")
from LASER_scanner.SDK.PYSDK_SMART import *
#from SDK.PYSDK_SMART import *

class LASER_scanner():
    def __init__(self,ip_addr_in='192.168.1.30'):
        self.scanner=None
        self.dump_length = 200
        self.dump_size=0
        self.scanning_freq = 100
        self.dump_fill_time=0
        self.recording = False
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
        self.realtime=False
        
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
                print("Scanner output datatype is not profile! Change output type")
                raise RuntimeError("Scanner output datatype is not profile! Change output type")
        else:
            print("Profile not recieved!")  
            raise RuntimeError("Profile not recieved!")
            
    def dump_record_start(self):
        if self.is_connected:
            print('trying to start recording')
            print(start_dump_recording(self.scanner,self.dump_length))
            self.recording = True
        else:
            raise RuntimeError('Not connected to scanner')
            
    def dump_full(self):
        print('dump_full_start')
        if self.is_connected:
            print('is connected')
            read_params(self.scanner)
            print('params read')
            self.dump_size = get_param(self.scanner, 'user_dump_size')['value']
            print('dump_size acquired')
            print(self.dump_size)
            self.dump_fill_time=(20-self.dump_size)/100
            print(self.dump_fill_time)
            return self.dump_size>=(self.dump_length-1)
        else:
            raise RuntimeError('Not connected to scanner')
    def get_dump(self):
        print('getting dump')
        print(self.dump_length)
        self.recording=False
        dump=get_dumps_profiles(self.scanner, 0, self.dump_length)
        print('dump acquired')
        print(dump)
        return dump
        
if __name__ == '__main__':
    """Test code:
      1. connect to scanner
      2. try getting a profile
      3. end connection
      
      Then do the same but for some time in order to get timing
      """
    import time  # for basic pausing (reliving CPU)
    print("Connect")
    scanner=LASER_scanner('192.168.1.30')
    print("Get_line")
    scanner.dump_record_start()
    print('sleep')
    time.sleep(3)
    print('try getting the dump shit')
    dump=scanner.get_dump()
    print(dump)
    prof = scanner.get_line()
    print("Try printing")
    #for point in prof:
    #    if point['z'] != 0.0:
    #        print('Z: ' + str(point['z']) + '   X: ' + str(point['x']))
    #print("Finished!")
    
    
    
    #print("Checking timing")
    if False:
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
        
