#!/usr/bin/env python2

import rospy
from driving_swarm_positioning.msg import Range
import numpy as np

class RingBuffer:
    """ class that implements a not-yet-full buffer """
    def __init__(self,size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """
        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max
        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self,x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data

class RangeBufferFilterDings:
    data = [] #[[start,ziel,buffer],[start,ziel,buffer],[start,ziel,buffer],...]
    window = 10
    def __init__(self, window=10):
        self.data = []
        self.window = window

    def addRange(self, msg):
        src = msg.src
        target = msg.dest
        range = msg.range
        for distancearray in self.data:
            if(distancearray[0] == src and distancearray[1] == target):
                distancearray[2].append(range)
                return distancearray
        g = RingBuffer(self.window)
        g.append(range)
        self.data.append([src, target, g])
        return self.data[len(self.data)-1]



def filterfunction(DataList):
    g = np.array(DataList)
    #print(g)
    return np.median(g);


rangefilter = RangeBufferFilterDings()
rangePub = rospy.Publisher('FilteredRangePublisher', Range, queue_size=10)


def callback(msg):
    global rangefilter
    lastValues = rangefilter.addRange(msg)[2].get()
    newRange = filterfunction(lastValues)
    newMsg = msg
    newMsg.range = newRange
    rangePub.publish(newMsg)


if __name__ == '__main__':
    global rangePub
    rospy.init_node("RangeFilter")
    medianFilterWindow = rospy.get_param('~MedianFilterWindow', 10)
    rangePub.window = medianFilterWindow
    #rangePub = rospy.Publisher('FilteredRangePublisher', Range, queue_size=10)
    rospy.Subscriber("/turtlebot1/RangePublisher", Range, callback)
    rospy.spin();
