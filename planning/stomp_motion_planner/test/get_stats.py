#!/usr/bin/env python
import roslib
roslib.load_manifest('stomp_motion_planner')
import rosbag
import sys
import math

bag=rosbag.Bag(sys.argv[1])
num_success=0
num_messages=0
success_duration=0.0
success_iteration=0.0
success_duration_std=0.0
success_iteration_std=0.0
for topic, msg, t in bag.read_messages():
  num_messages = num_messages + 1
  if msg.success:
    num_success = num_success + 1
    success_duration = success_duration + msg.success_duration
    success_iteration = success_iteration + float(msg.success_iteration)
bag.close()

success_percentage = float(num_success) / float(num_messages)
success_duration = success_duration / float(num_success)
success_iteration = success_iteration / float(num_success)

# second pass for std_dev:
bag=rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages():
  if msg.success:
    delta = msg.success_duration - success_duration
    success_duration_std = success_duration_std + delta*delta
    delta = msg.success_iteration - success_iteration
    success_iteration_std = success_iteration_std + delta*delta
bag.close()

success_duration_std = math.sqrt(float(success_duration_std)/num_success)
success_iteration_std = math.sqrt(float(success_iteration_std)/num_success)

print 'Number of successes: ',num_success,'/',num_messages
print 'Success percentage: ', success_percentage
print 'Avg success duration: ', success_duration, '+-', success_duration_std
print 'Avg success iterations: ', success_iteration, '+-', success_iteration_std
