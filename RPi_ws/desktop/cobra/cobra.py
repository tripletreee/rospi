from smbus2 import SMBus
import time
import SocketServer


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
        #print self.data
        speed = self.data.split(':')
        speed_left = int(100*float(speed[0]))
        speed_right = int(100*float(speed[1]))
        Cobra.i2c.write_byte(0x41, speed_left)
        Cobra.i2c.write_byte(0x42, speed_right)
        # just send back the same data, but upper-cased
        #self.request.sendall(self.data.upper())


class Cobra:
    def __init__(self):
        ''' Creates a object 
        '''
        self.HOST = '192.168.0.102'
        self.PORT = 2017
        self.server = SocketServer.TCPServer((self.HOST, self.PORT), TCPHandler)
        
        self.i2c = SMBus(1)
        self.i2c.write_byte(0x40, 0)
        self.i2c.write_byte(0x41, 0)
        self.i2c.write_byte(0x42, 0)
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        self.server.serve_forever()




    
'''
i2c = SMBus(1)

result = i2c.write_byte(0x40, 0)

result = i2c.write_byte(0x41, 5)

result = i2c.write_byte(0x42, 5)

print(result)

i2c.close()
'''