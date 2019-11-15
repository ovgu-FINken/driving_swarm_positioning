#!/usr/bin/env python3

import rospy
from driving_swarm_positioning.msg import Range
import serial
import pprz.py

module = serial.Serial("/dev/ttyS0", 115200)

while True:
   print(f"Line: {module.readln()}")


