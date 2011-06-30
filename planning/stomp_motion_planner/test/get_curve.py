#!/usr/bin/env python
import roslib
roslib.load_manifest('stomp_motion_planner')
import rosbag
import sys
import math

f = open('curves.txt','w')
bag=rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages():
  f.write('\t'.join(str(d) for d in msg.costs) + '\n')
bag.close()
f.close()
