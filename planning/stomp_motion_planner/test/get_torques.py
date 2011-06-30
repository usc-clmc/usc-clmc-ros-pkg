#!/usr/bin/env python
import roslib
roslib.load_manifest('stomp_motion_planner')
import rosbag
import sys
import math

f = open('torques.txt','w')

bag=rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages():
  f.write('\t'.join(str(d) for d in msg.torques) + '\n')
bag.close()

bag=rosbag.Bag(sys.argv[2])
for topic, msg, t in bag.read_messages():
  f.write('\t'.join(str(d) for d in msg.torques) + '\n')
bag.close()

f.close()
