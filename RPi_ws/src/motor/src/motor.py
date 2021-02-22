#!/usr/bin/env python
import rospy
from smbus2 import SMBus
from geometry_msgs.msg import Twist

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    left = data.linear.x - data.angular.z
    right = data.linear.x + data.angular.z
    i2c = SMBus(1)
    i2c.write_byte(0x40, 101)
    i2c.write_byte(0x40, int(left))
    i2c.write_byte(0x40, 102)
    i2c.write_byte(0x40, int(right))
    i2c.close()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("motorSpeeds", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

