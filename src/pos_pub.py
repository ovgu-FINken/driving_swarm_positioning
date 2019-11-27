#!/usr/bin/env python2

import rospy
from driving_swarm_positioning.msg import Range
import time
import numpy as np

class RangeBuffer:
    data=[]
    anchors={}
    def __init__(self):
        self.data=[]

    def addRange(self, src, target, range):
        for rangeFrame in data:
            if(rangeFrame.src == src and rangeFrame.target == target):
                rangeFrame.update(range)
                return
        if(str(target) in self.anchors):
            self.data.append(RangeFrame(src, target, range, self.anchors[str(target)]))
        else:
            self.data.append(RangeFrame(src, target, range, None))

    def addRange(self, msg):
        src = msg.src
        target = msg.dest
        range = msg.range
        for rangeFrame in self.data:
            if(rangeFrame.src == src and rangeFrame.target == target):
                rangeFrame.update(range)
                return
        if(str(target) in self.anchors):
            self.data.append(RangeFrame(src, target, range, self.anchors[str(target)]))
        else:
            self.data.append(RangeFrame(src, target, range, None))

    def printBuffer(self):
        for rangeFrame in self.data:
            print(str(rangeFrame.src) + " --> " + str(rangeFrame.target) + " = " + str(rangeFrame.range) + "   : " + str(rangeFrame.howold()) + "       " + str(rangeFrame.anchorPos) )

class RangeFrame:
    range = None
    src = None
    target = None
    anchorPos = None # probably unnecessary
    lastUpdate = None
    def __init__(self, src, target, range, anchorPos):
        self.range = range
        self.src = src
        self.target = target
        self.lastUpdate = time.time()
        self.anchorPos = anchorPos

    def update(self, range):
        self.range = range
        self.lastUpdate = time.time()

    def howold(self):
        return (time.time() - self.lastUpdate)


rangebuffer = RangeBuffer()
latestPosition = np.array([0,0,0.10])
maxAge = 2.

def build_pos_msg(pos):
    return

def compute_G(point, anchors):
    G = []
    for i in range(len(anchors)):
        R_i = np.linalg.norm(point - anchors[i])
        G.append((point - anchors[i]) / R_i)
    return np.array(G)

def compute_position(rb, max_iterations=10, startpoint=[0,0,0]):
    anchors = []
    distances = []
    #fill the arrays so that the order is the same but independent of sorting
    for rf in rb.data:
        if rf.anchorPos != None and rf.howold() < maxAge :    # ignore distances to points with unknown positions and to old distances
            anchors.append(rf.anchorPos)
            distances.append(rf.range)

    iter_point = startpoint

    for g in range(0,max_iterations):
        current_distances = np.array([np.linalg.norm(a - iter_point) for a in anchors])
        G = compute_G(iter_point, anchors)
        iter_point = iter_point + 0.5 * np.matmul(np.linalg.pinv(G),(distances - current_distances))

    return iter_point

def callback(msg):
    #print("received Message:")
    #print(msg.range)
    rangebuffer.addRange(msg);
    #rangebuffer.printBuffer();
    #pos = compute_position(rangebuffer, startpoint = latestPosition)
    #print(pos)

def range_subscriber():
    if rospy.search_param('anchors'):
        param_name = rospy.search_param('anchors')
        anchors = rospy.get_param(param_name)
        rangebuffer.anchors = anchors
    else:
        raise NameError('no anchors defined')

    updateRate = 0 #Hz

    if rospy.search_param('positionUpdateRate'):
        param_name = rospy.search_param('positionUpdateRate')
        updateRate = rospy.get_param(param_name)
    else:
        raise NameError('no update Rate defined')

    rospy.init_node("range_subscriber")
    rospy.Subscriber("/turtlebot3/RangePublisher", Range, callback)
    rate = rospy.Rate(updateRate)
    while not rospy.is_shutdown():
        if (len(rangebuffer.data) > 4):
        	pos = compute_position(rangebuffer, startpoint = latestPosition)
        	print(pos)
        rate.sleep()
    # rospy.spin()
    # repeate in given rate :
        # update position (also latest position as starting point)

if __name__ == '__main__':
    range_subscriber()
