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

'''this node calculates 2d positions based on RangeMessages from Targets to Anchornodes. 
The Anchornodes + UpdateRate are defined in param/RangePositionParameters.yml '''
# Input:    RangePositionParameters.yml, RangeDistanceMessages
# Output:   Transformation(static, world->locSystem), Transformations(nonstatic, locSystem->Targets)

locSystemName = "UWBLocalisation"

class RangeBuffer:
    '''Construct contains array of Rangeframes '''
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
    '''Construct contains Range/Distance, Source, Target/Destination of the Distance and lastUpdateTime'''
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
    '''Sends static transformation from world->locSystem'''
    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'world'
    t.child_frame_id = locSystemName
    t.transform.translation.x = 0.0  
    t.transform.translation.y = 0.0  
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)


def send_target_transform(br, target, pos):
    '''Sends nonstatic transformations from locSystem->Targets'''
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = locSystemName
    t.child_frame_id = locSystemName + "/target" + str(target)
    t.transform.translation.x = pos[target][0] 
    t.transform.translation.y = pos[target][1] 
    t.transform.translation.z = pos[target][2]
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)


def compute_G(point, anchors):
    '''compute gradient-matrix for iterrative position solving'''
    G = []
    for i in range(len(anchors)):
        R_i = np.linalg.norm(point - anchors[i])
        G.append((point - anchors[i]) / R_i)
    return np.array(G)

def compute_multible_positions(rb, max_iterations=10):
    '''compute positions of all known targets (targets known through source in Range-messages)'''
    targetList = []
    targets = {}
    for rf in rb.data:
        if rf.src not in targetList:
            targetList.append(rf.src)
    for t in targetList:
        target = compute_position(rb, t, max_iterations)
        targets[t] = target
    return targets

def compute_position(rb, targetNR, max_iterations=10, startpoint=np.array([0,0,0])):
    '''iterative solver for a single target'''
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

    if len(distances)<3:
        return None

    for g in range(0,max_iterations):
        current_distances = np.array([np.linalg.norm(a - iter_point) for a in anchors])
        G = compute_G(iter_point, anchors)
	d = np.matmul(np.linalg.pinv(G),(distances - current_distances))
        iter_point = iter_point + 0.5 * np.array([d[0], d[1], 0]) # only calculate 2 dimensional position

    return iter_point


def callback(msg):
    '''adds new received RangeMessage to the RangeBuffer'''
    #print("received Message:")
    #print(msg.range)
    rangebuffer.addRange(msg)
    rangebuffer.printBuffer()
    #pos = compute_position(rangebuffer, startpoint = latestPosition)
    #print(pos)

if __name__ == '__main__':
    '''Init RosNode, publisher, subscriber + load parameters and fill Rangebuffer with given anchornodes'''
    rospy.init_node("RangePositionCalculator")
    anchors = rospy.get_param('~anchors')
    rangebuffer.anchors = anchors        
    updateRate = rospy.get_param('~positionUpdateRate', 10) #Hz
    rangeTimeout = rospy.get_param('~rangeTimeout', 2.) #s
    RangeMessageTopic = rospy.get_param('~RangeMessageTopic', "/FilteredRangePublisher")

    rospy.loginfo("i am running NOW")

    send_static_map_transform()

    rospy.Subscriber(RangeMessageTopic, Range, callback)
    
    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(updateRate)
    while not rospy.is_shutdown():
        if (len(rangebuffer.data) > 1): # avoid empty buffer error 
            pos = compute_multible_positions(rangebuffer) #todo latest position array as startpoints of iterative position solver !!
            rospy.loginfo(pos)
            print(pos)

            for singlepos in pos: # create transformations for all the tragets
                try: 
                    if pos[singlepos] != None:
                        send_target_transform(br, singlepos, pos)
                except:
                    send_target_transform(br, singlepos, pos)

        rate.sleep()
