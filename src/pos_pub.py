#!/usr/bin/env python2

import rospy
from driving_swarm_positioning.msg import Range
import time

class RangeBuffer:
    data=[]
    def __init__(self):
        self.data=[]

    def addRange(self, src, target, range):
        for rangeFrame in data:
            if(rangeFrame.src == src and rangeFrame.target == target):
                rangeFrame.range = range
                break
        data.append(RangeFrame(src, target, range))

    def addRange(self, msg):
        src = msg.src
        target = msg.dest
        range = msg.range
        for rangeFrame in data:
            if(rangeFrame.src == src and rangeFrame.target == target):
                rangeFrame.range = range
                break
        data.append(RangeFrame(src, target, range))

    def printBuffer(self):
        for rangeFrame in self.data:
            print(str(rangeFrame.src) + " --> " + str(rangeFrame.target) + " = " + str(rangeFrame.range))

class RangeFrame:
    range = Null
    src = Null
    target = Null
    lastUpdate = Null
    def __init__(self, src, target, range):
        self.range = range
        self.src = src
        self.target = target
        self.lastUpdate = time.time()

    def update(self, range):
        self.range = range
        self.time = time.time()

    def howold(self):
        return (time.time() - self.lastUpdate)

RangeBuffer rangebuffer()

def callback(msg):
    print("received Message:")
    print(msg.range)
    rangebuffer.add(msg);
    rangebuffer.printBuffer();

def range_subscriber():
    rospy.init_node("range_subscriber")
    rospy.Subscriber("/turtlebot3/RangePublisher", Range, callback)
    rospy.spin()

if __name__ == '__main__':
    range_subscriber()
