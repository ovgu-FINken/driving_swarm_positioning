#!/usr/bin/env python3

#import rospy
#from driving_swarm_positioning.msg import Range
import serial
import pprz

module = serial.Serial("/dev/ttyS0", 115200)
parser = pprz.PprzParser(callback=pprz.parse_range)


while True:
  parser.data_in(module.read())






