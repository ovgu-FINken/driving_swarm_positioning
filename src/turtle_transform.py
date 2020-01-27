#!/usr/bin/env python2

import rospy
from driving_swarm_positioning.msg import Range
import serial
import pprz

'''gets the module id from the serial port and publishes a 0-transformation to the uwb_loc_system'''
# Input:    Received Serial Data
# Output:   Transformation to uwb_loc_system

locSystemName = "loc_system_uwb"

module = serial.Serial("/dev/ttyUSB0", 115200) # change depending on the port the module is connected to
parser = pprz.PprzParser()


def send_static_transform(id_nr):
    '''Sends static transformation from world->locSystem'''
    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'locSystemName + "/target" + str(id_nr)'
    t.child_frame_id = 'tb3_1' # todo: make variable / read existing global variable ? maybe namespace prefix in which this is run
    t.transform.translation.x = 0.0  
    t.transform.translation.y = 0.0  
    t.transform.translation.z = 0.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

    br.sendTransform(t)



def talker():
    pub = rospy.Publisher('RangePublisher', Range, queue_size=100)
    rospy.init_node('RangePublisher')
    rate = rospy.Rate(0.2) 
    while not rospy.is_shutdown():
        g = module.read()
        pkg = parser.data_in(g)
        if(pkg):
            dist, src, dest = pprz.parse_range(pkg)
            ModuleID = src
            send_static_transform(src)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
