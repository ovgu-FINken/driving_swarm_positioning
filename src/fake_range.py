#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from driving_swarm_positioning.msg import Range

def sendRanges(pub, anchors, targets):
    for i in range(0,len(targets)-1):
        for anchor in anchors:
            rangeMsg = Range()
            rangeMsg.src = i
            rangeMsg.dest = int(anchor) 
            #rospy.logerr(targets[i])
            #rospy.logerr(anchor)
            rangeMsg.range = np.linalg.norm(np.array(targets[i])-np.array(anchors[anchor]))
            pub.publish(rangeMsg)
            print(rangeMsg)
    return 


if __name__ == '__main__':
    pub = rospy.Publisher('RangePublisher', Range, queue_size=100)
    rospy.init_node('RangePublisher') 
    anchors = rospy.get_param('~anchors')
    updatRate = rospy.get_param('~UpdateRate')
    targets = rospy.get_param('~Targets')
    rate = rospy.Rate(updatRate)
    while not rospy.is_shutdown():
        sendRanges(pub, anchors, targets)
        rate.sleep()

    rospy.spin()
