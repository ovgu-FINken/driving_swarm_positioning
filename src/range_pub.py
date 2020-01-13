#!/usr/bin/env python2

import rospy
from driving_swarm_positioning.msg import Range
import serial
import pprz

module = serial.Serial("/dev/ttyUSB0", 115200)
parser = pprz.PprzParser()


def talker():
    pub = rospy.Publisher('RangePublisher', Range, queue_size=100)
    rospy.init_node('RangePublisher') 
    while not rospy.is_shutdown():
        g = module.read()
        pkg = parser.data_in(g)
        if(pkg):
            dist, src, dest = pprz.parse_range(pkg)
            rangeMsg = Range()
            rangeMsg.src = src
            rangeMsg.dest = dest 
            rangeMsg.range = dist
            if src==pkg.sender_id:
                pub.publish(rangeMsg)
                print("sent Range Message")
            else:
                print("kapuuuttt")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
