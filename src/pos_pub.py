#!/usr/bin/env python2

import rospy
from driving_swarm_positioning.msg import Range
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import time
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg

locSystemName = "UWBLocalisation"

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
            rospy.logdebug(str(rangeFrame.src) + " --> " + str(rangeFrame.target) + " = " + str(rangeFrame.range) + "   : " + str(rangeFrame.howold()) + "       " + str(rangeFrame.anchorPos) )

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
rangeTimeout = 2.


def send_static_map_transform():
    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = locSystemName
    t.transform.translation.x = 0.0  # msg.origin.position.x
    t.transform.translation.y = 0.0  # msg.origin.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)


def compute_G(point, anchors):
    G = []
    for i in range(len(anchors)):
        R_i = np.linalg.norm(point - anchors[i])
        G.append((point - anchors[i]) / R_i)
    return np.array(G)

def compute_multible_positions(rb, max_iterations=10, startpoint=[0,0,0]):
    targetList = []
    targets = {}
    for rf in rb.data:
        if rf.src not in targetList:
            targetList.append(rf.src)
    for t in targetList:
        target = compute_position(rb, t, max_iterations, startpoint)
        targets[t] = target
    return targets

def compute_position(rb, targetNR, max_iterations=10, startpoint=np.array([0,0,0])):

    anchors = []
    distances = []
    #fill the arrays so that the order is the same but independent of sorting
    for rf in rb.data:
        if rf.anchorPos != None and rf.howold() < rangeTimeout and rf.src == targetNR :    # ignore distances to points with unknown positions and to old distances
            anchors.append(rf.anchorPos)
            distances.append(rf.range)

    anchors = np.array(anchors)
    distances = np.array(distances)
    iter_point = startpoint

    for g in range(0,max_iterations):
        current_distances = np.array([np.linalg.norm(a - iter_point) for a in anchors])
        G = compute_G(iter_point, anchors)
	d = np.matmul(np.linalg.pinv(G),(distances - current_distances))
        iter_point = iter_point + 0.5 * np.array([d[0], d[1], 0]) # only calculate 2 dimensional position

    return iter_point

def createPosMsg(pos):
    posearraymsg = PoseArray()
    for singlepos in pos:
        posemsg = Pose()
        posemsg.position.x = pos[singlepos][0]
        posemsg.position.y = pos[singlepos][1]
        posemsg.position.z = pos[singlepos][2]
        posearraymsg.poses.append(posemsg)
    return posearraymsg

def callback(msg):
    #print("received Message:")
    #print(msg.range)
    rangebuffer.addRange(msg)
    rangebuffer.printBuffer()
    #pos = compute_position(rangebuffer, startpoint = latestPosition)
    #print(pos)

if __name__ == '__main__':

    rospy.init_node("RangePositionCalculator")
    anchors = rospy.get_param('~anchors')
    rangebuffer.anchors = anchors        
    updateRate = rospy.get_param('~positionUpdateRate', 10) #Hz
    rangeTimeout = rospy.get_param('~rangeTimeout', 2.) #s

    rospy.loginfo("i am running NOW")

    send_static_map_transform()

    posPub = rospy.Publisher('RangePositionPublisher', PoseArray, queue_size=10) 
    rospy.Subscriber("/FilteredRangePublisher", Range, callback)
    rate = rospy.Rate(updateRate)
    while not rospy.is_shutdown():
        if (len(rangebuffer.data) > 4): #todo (auslagern + timeout beruecksichtigen!) kann hier so weg, macht durch mehrere targets im gleichen buffer keinen sinn mehr 
            #rospy.loginfo("debug: entered rate loop")
            pos = compute_multible_positions(rangebuffer) #todo latest position array als startpoints !!
            #rospy.loginfo("debug: past compute_multible_positions")
            rospy.loginfo(pos)
            print(pos)
            posPub.publish(createPosMsg(pos))
        rate.sleep()
