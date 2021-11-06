# from ..MPU_9255_I2C. import MPU_9255_reader
from rp_client.MPU_9255_I2C.MPU_9255_reader import MPU_9255_reader
from rp_client.ISM330DLC_reading_SPI.ISM330DLC_reader import ISM_330_reader

sens_1 = MPU_9255_reader()
sens_2 = ISM_330_reader()
