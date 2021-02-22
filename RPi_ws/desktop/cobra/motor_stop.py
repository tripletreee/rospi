from smbus2 import SMBus

i2c = SMBus(1)

result = i2c.write_byte(0x40, 101)

result = i2c.write_byte(0x40, 0)

result = i2c.write_byte(0x40, 102)

result = i2c.write_byte(0x40, 0)

i2c.close()
