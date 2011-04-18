#!/usr/bin/env python
# Converts a pi2_statistics.bag file to a text file with the learning curve
import roslib
roslib.load_manifest('policy_improvement_loop')
import rosbag
import sys
import math

f = open('learning_curve.txt','w')
bag=rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages():
  f.write(str(msg.noiseless_cost) + '\n')
bag.close()
f.close()
