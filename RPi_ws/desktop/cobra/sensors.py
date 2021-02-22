from smbus2 import SMBus
import lis3mdl
import lsm6ds33
import lps25hb

class Sensors:

	mag     =  [0, 0, 0]
	acc     =  [0, 0, 0]
	gyro    =  [0, 0, 0]
	pres	=  0
	temp    =  0
	
	def __init__(self):
	
		self.i2c = SMBus(1)
		
	def shutdown(self):
	
		self.i2c.close()
		
	def boot(self):
			
		# Initial LIS3MDL: 3-axis magnetometer
		# OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
		self.i2c.write_byte_data(lis3mdl.ADDRESS, lis3mdl.CTRL_REG1, 0b01110000)
		# FS = 00 (+/- 4 gauss full scale)
		self.i2c.write_byte_data(lis3mdl.ADDRESS, lis3mdl.CTRL_REG2, 0b00000000)
		# MD = 00 (continuous-conversion mode)
		self.i2c.write_byte_data(lis3mdl.ADDRESS, lis3mdl.CTRL_REG3, 0b00000000)
		# OMZ = 11 (ultra-high-performance mode for Z)
		self.i2c.write_byte_data(lis3mdl.ADDRESS, lis3mdl.CTRL_REG4, 0b00001100)
		
		# Initial LSM6DS33: 3-axis gyro and 3-aixs accelerometer
		# LSM6DS33 gyro
		# ODR = 1000 (1.66 kHz (high performance))
		# FS_G = 11 (2000 dps)
		self.i2c.write_byte_data(lsm6ds33.ADDRESS, lsm6ds33.CTRL2_G, 0b10001100)
		# defaults
		self.i2c.write_byte_data(lsm6ds33.ADDRESS, lsm6ds33.CTRL7_G, 0b00000000)
		# LSM6DS33 accelerometer
		# ODR = 1000 (1.66 kHz (high performance))
		# FS_XL = 11 (8 g full scale)
		# BW_XL = 00 (400 Hz filter bandwidth)
		self.i2c.write_byte_data(lsm6ds33.ADDRESS, lsm6ds33.CTRL1_XL, 0b10001100)
		# common
		# IF_INC = 1 (automatically increment address register)
		self.i2c.write_byte_data(lsm6ds33.ADDRESS, lsm6ds33.CTRL3_C, 0b00000100)
		
		# Initial LPS25HB: Pressure and Temperature
		# ODR = 001 (1Hz)
		self.i2c.write_byte_data(lps25hb.ADDRESS, lps25hb.CTRL_REG1, 0b10010000)
		
		
		
	def read_mag (self):
		
		# Read a block of 6 bytes from the magnetometer
		block = self.i2c.read_i2c_block_data(lis3mdl.ADDRESS, 0x80 | lis3mdl.OUT_X_L, 6)
		self.mag[0] = self.word_to_int(block[0] | block[1] << 8)
		self.mag[1] = self.word_to_int(block[2] | block[3] << 8)
		self.mag[2] = self.word_to_int(block[4] | block[5] << 8)
		
	def read_acc (self):
		
		# Read a block of 6 bytes from the accelerometer
		block = self.i2c.read_i2c_block_data(lsm6ds33.ADDRESS, lsm6ds33.OUTX_L_XL, 6)
		self.acc[0] = self.word_to_int(block[0] | block[1] << 8)
		self.acc[1] = self.word_to_int(block[2] | block[3] << 8)
		self.acc[2] = self.word_to_int(block[4] | block[5] << 8)
		
	def read_gyro (self):
		
		# Read a block of 6 bytes from the accelerometer
		block = self.i2c.read_i2c_block_data(lsm6ds33.ADDRESS, lsm6ds33.OUTX_L_G, 6)
		self.gyro[0] = self.word_to_int(block[0] | block[1] << 8)
		self.gyro[1] = self.word_to_int(block[2] | block[3] << 8)
		self.gyro[2] = self.word_to_int(block[4] | block[5] << 8)
	
	def read_pres_temp (self):
		
		# Read a block of 5 bytes from the sensor
		block = self.i2c.read_i2c_block_data(lps25hb.ADDRESS, lps25hb.PRESS_OUT_XL | 0x80, 5)
		self.pres = (block[0] | block[1] << 8 | block[2] << 16)/4096
		self.temp = self.word_to_int(block[3] | block[4] << 8)
                self.temp = 42.5+self.temp/480

	def read_data (self):
		
		self.read_mag()
		self.read_acc()
		self.read_gyro()
		self.read_pres_temp()
		
        def word_to_int (self, word):

                return word if word < 32768 else (word - 65536)
                
		
