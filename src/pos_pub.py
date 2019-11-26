#!/usr/bin/env python2 

import rospy 
from driving_swarm_positioning.msg import Range

class RangeBuffer:
    data=[] 
    def __init__(self):
        self.data=[][]



def callback(msg):
    print("received Message:")
    print(msg.range)

def range_subscriber():
    rospy.init_node("range_subscriber") 
    rospy.Subscriber("/turtlebot3/RangePublisher", Range, callback)
    rospy.spin()

if __name__ == '__main__':
    range_subscriber() 
