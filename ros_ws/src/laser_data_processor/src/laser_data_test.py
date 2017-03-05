#!/usr/bin/env python
# license removed for brevity
import rospy
import PIL as Image
import numpy as np
from math import *
from sensor_msgs.msg import LaserScan

def callback(data):
    #im = np.full((100,100),255)
    
    print((abs(data.angle_min) + data.angle_max)/data.angle_increment)
    
    #im = Image.fromarray(Matrix)
    #im = im.convert('RGB')
    #im.save("laser_data.png")
    
def listener():
    
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
