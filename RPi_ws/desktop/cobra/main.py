from smbus2 import SMBus
import time
import SocketServer


HOST = '192.168.1.102'
#HOST = '10.214.96.19'
PORT = 2017

class TCPHandler(SocketServer.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024)
        print "{} wrote:".format(self.client_address[0])
        
        speed = self.data.split(':')
        
        speed_left = int(100*float(speed[0]))
        speed_right = int(100*float(speed[1]))

        if speed_left < 0:
        	speed_left = 256+speed_left

        if speed_right < 0:
        	speed_right = 256+speed_right;

        print (speed_left,speed_right)
        i2c = SMBus(1)
        i2c.write_byte(0x41, speed_left)
        i2c.write_byte(0x42, speed_right) 
        i2c.close()


server = SocketServer.TCPServer((HOST, PORT), TCPHandler)
i2c = SMBus(1)
i2c.write_byte(0x40, 1)
i2c.write_byte(0x41, 0)
i2c.write_byte(0x42, 0)
i2c.close()

server.serve_forever()





    
'''
i2c = SMBus(1)

result = i2c.write_byte(0x40, 0)

result = i2c.write_byte(0x41, 5)

result = i2c.write_byte(0x42, 5)

print(result)

i2c.close()
'''
