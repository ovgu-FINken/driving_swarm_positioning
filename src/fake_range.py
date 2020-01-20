#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from driving_swarm_positioning.msg import Range

'''this node sends ranges from the anchor nodes to fixed points defined in 
param/RangePositionParameters.yml (rosparam load RangePositionParameters)'''
# Input:    RangePositionParameters.yml
# Output:   RangeDistanceMessages from virtual Targets to Anchor-Nodes


def sendRanges(pub, anchors, targets):
    '''calculate distances from anchors to targets and publish ranges'''
    for i in range(0,len(targets)):
        for anchor in anchors:
            rangeMsg = Range() 
            rangeMsg.src = i
            rangeMsg.dest = int(anchor)
            rangeMsg.range = np.linalg.norm(np.array(targets[i])-np.array(anchors[anchor]))
            pub.publish(rangeMsg)
            print(rangeMsg)
    return 


if __name__ == '__main__':
    '''initialize rosnode, publisher + read parameters'''
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
