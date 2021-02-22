from smbus2 import SMBus

import time

import math

i2c = SMBus(1)

battery_voltage_highbyte = i2c.read_byte_data(0x48, 1);

battery_voltage_lowbyte = i2c.read_byte_data(0x48, 2);

battery_voltage = battery_voltage_highbyte*256 + battery_voltage_lowbyte;

print('Battery voltage', float(battery_voltage)*9.682/1024)

battery_temperature_highbyte = i2c.read_byte_data(0x48, 3);

battery_temperature_lowbyte = i2c.read_byte_data(0x48, 4);

v_t = battery_temperature_highbyte*256 + battery_temperature_lowbyte;

v_t = float(v_t)/1024*2.5

R_t = v_t*220000/(3.3-v_t)

temperature = 1/(math.log(R_t/100000)/4250+0.003355704698)-273

print('Temperature', temperature)

adapter_voltage_highbyte = i2c.read_byte_data(0x48, 5);

adapter_voltage_lowbyte = i2c.read_byte_data(0x48, 6);

adapter_voltage = adapter_voltage_highbyte*256 + adapter_voltage_lowbyte;

print('Adapter voltage', float(adapter_voltage)*15.25/1024)

charging_mode = i2c.read_byte_data(0x48, 9);

print('Charging mode', charging_mode)

battery_mode = i2c.read_byte_data(0x48, 10);

print('Battery mode', battery_mode)

i2c.close()
