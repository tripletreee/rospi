from sensors import Sensors
import time
 
sensors = Sensors()
sensors.boot()

 
while True:
	
       sensors.read_data()
       print(sensors.mag, sensors.acc, sensors.gyro)
       time.sleep(0.02)
 
sensors.close()
print("sensor closed");
