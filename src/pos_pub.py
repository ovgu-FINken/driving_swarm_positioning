#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
import serial
import numpy as np

class Package:
    def __init__(self, sender_id=0, msg_id=0, payload=[]):
        self.startbyte = 0x99
        # payload + start, len, msg_id, sender_id + checkSumA, B
        self.length = len(payload) + 6
        self.sender_id = sender_id
        self.msg_id = msg_id
        self.payload = payload

    @classmethod
    def from_pprz(cls, pprz):
        if len(pprz) < 6:
            logging.debug("Package to small")
        if pprz[0] != 0x99:
            logging.debug("No startbyte found in pprz string")
            return None
        pkg = Package()
        pkg.length = pprz[1]
        pkg.sender_id = pprz[2]
        pkg.msg_id = pprz[3]
        pkg.payload = pprz[4:-2]
        pkg.checkSumA = pprz[-2]
        pkg.checkSumB = pprz[-1]
        return pkg


class Pub:
    def __init__(self):
        self.pos_pub = rospy.Publisher("position", Point, queue_size=1)
        self.range_pub = rospy.Publisher("ranges", Float32MultiArray, queue_size=1)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.ser.flush()
        self.data = bytes([])
        self.pos = [0.0,0.0,0.0]
        self.anchors = np.array([[0,0.15,2.999],[4.406,0.192,2.999],[0.685,0.22,0.115],[0.658,2.895,0.125],[4.92,0.145,0.22],[4.92,2.96,0.22], [4.287,3.804,1.154], [-2.345,6.005,2.55], [-2.073,3.203,2.553]])
        self.last_result = {}
        self.pkg = Package()
        
    def write(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            tmp = self.ser.read(self.ser.in_waiting)
            print(tmp)
            Pub.data_in(tmp)
            src, dest, dist = Pub.parse_range()
            self.pos = range_command()
            if data:
                self.last_result[f"{dest} -> {dist}"] = src

            out_range = Float32MultiArray()
            out_range.dim.append(MultiArrayDimension())
            out_range.dim.append(MultiArrayDimension())
            
            out_range.dim[0].label = "rangeID"
            out_range.dim[1].label = "rangeValue"
            out_range.dim[0].stride = 1
            out_range.dim[1].stride = 1
            out_range.data_offset = 0
            out_range.data = [0]*len(last_result)*2
            out_range.dim[0].size = len(last_result)
            out_range.dim[1].size = len(last_result)

            val = 0
            for k, v in sorted(last_result.items()):
                out_range.data[val]=k
                val = val + 1
                out_range.data[val]=v
                val = val + 1
            self.range_pub.publish(out_range)
            out = Point()
            out.x = pos[0]
            out.y = pos[1]
            out.z = pos[2]
            self.pub.publish(out)
            rate.sleep()

    def data_in(self, data):
        self.data = self.data + bytes(data)
        """
        while(len(self.data) > 6):
            if self.data[0] != 0x99:
                self.data = self.data[1:]
            else:
                break
        
        if len(self.data) < 6:
            return
        if len(self.data) < self.data[1]:
            return
        """
        self.pkg = Package.from_pprz(self.data[:self.data[1]])
        self.data = self.data[self.data[1]:]

    def parse_range(self):
        if self.pkg == None:
            return None
        if self.pkg.msg_id == 254:
            try:
                src, dest, dist = struct.unpack("=BBd", self.pkg.payload)
                return dist, src, dest
            except:
                return None
        return None

median_result = {}
def median_filter():
    global median_result
    for k, v in sorted(last_result.items()):
        if k in median_result.keys():
            if v not in median_result[k]:
                if len(median_result[k])>=MEDIANFILTER: #it was 10
                    median_result[k].popleft()
                median_result[k].append(v)
        else:
            median_result[k] = deque([v])
    #left_buffer.text += str(median_result)
    return {k : median(v) for (k,v) in median_result.items() }



def range_command():
    global rangeID
    global numberAnchors
    global anchors
    distances = []
    median_distances = median_filter() # medianfilter (auskommentierbar)
    for i in range(128, 128 + numberAnchors):
        if (f"{rangeID} -> {i}") in median_distances.keys(): # medianfilter --> nicht so einfach auskommentierbar
            distances.append(median_distances[f"{rangeID} -> {i}"])
        else:
            distances.append(0)
        
    solution = ra.iterative_solver_V2(anchors, np.array(distances), ITERATIONS, False, startpoint=last_position)
    return solution

if __name__ == '__main__':
    rospy.init_node('position', anonymous=True)
    Pub = Pub()
    try:
        Pub.write()
    except rospy.ROSInterruptException:
        pass
