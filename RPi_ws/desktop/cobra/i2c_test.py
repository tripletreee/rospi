from smbus2 import SMBus

import time

import math

i2c = SMBus(1)

result = i2c.write_byte(0x40, 102);

result = i2c.write_byte(0x40,0);


print(result)

i2c.close()
