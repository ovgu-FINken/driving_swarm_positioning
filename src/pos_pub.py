#!/usr/bin/python3

import rospy

class PosPub:
    def __init__(self):
        self.pub = rospy.Publisher("position", Point, queue_size=1)
    def publish:
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            #doshit
            out = Point
            out.x = 0
            out.y = 0
            out.z = 0
            self.pub.publish(out)
            rate.sleep()


if __name__ = '__main__':
    rospy.init_node('position', anonymous=True)
    PosPub = PosPub()
    try:
        PosPub.publish()
    except rospy.ROSInterruptException:
        pass
