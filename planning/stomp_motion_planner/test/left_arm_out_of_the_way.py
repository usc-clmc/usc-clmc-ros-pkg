#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# 
# Code stolen from tuckarm.py

import roslib
import signal
roslib.load_manifest('pr2_tuckarm')

import rospy
import os

from trajectory_msgs.msg import *
from pr2_mechanism_msgs.msg import *
from pr2_mechanism_msgs.srv import *

import sys
import time

name_right = 'r_arm_controller'
name_left = 'l_arm_controller'
pub_right = rospy.Publisher('%s/command'%name_right, JointTrajectory, latch=True)
pub_left = rospy.Publisher('%s/command'%name_left, JointTrajectory, latch=True)
pub_torso = rospy.Publisher('torso_controller/command', JointTrajectory, latch=True)

USAGE = 'tuckarm.py <arms> ; <arms> is \'(r)ight\', \'(l)eft\', or \'(b)oth\' arms'

prev_handler = None
resp =  SwitchControllerResponse()
stop_list = []

def torso_up():
  traj = JointTrajectory()
  traj.joint_names = ["torso_lift_joint"]
  traj.points = []
  
  for p in positions:
    traj.points.append(JointTrajectoryPoint(positions = [0.305],
                                            velocities = [0.0],
                                            accelerations = [0.0],
                                            time_from_start = rospy.Duration(10.0)))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)

  pub_torso.publish(traj)


def go(side, positions):
  traj = JointTrajectory()
  traj.joint_names = ["%s_shoulder_pan_joint" % side, 
                      "%s_shoulder_lift_joint" % side,
                      "%s_upper_arm_roll_joint" % side,
                      "%s_elbow_flex_joint" % side, 
                      "%s_forearm_roll_joint" % side,
                      "%s_wrist_flex_joint" % side, 
                      "%s_wrist_roll_joint" % side]
  traj.points = []

  for p in positions:
    traj.points.append(JointTrajectoryPoint(positions = p[1:],
                                            velocities = [0.0] * (len(p) - 1),
                                            accelerations = [],
                                            time_from_start = rospy.Duration(p[0])))

  traj.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)

  {'l': pub_left, 'r': pub_right}[side].publish(traj)



if __name__ == '__main__':
  rospy.init_node('tuck_arms', anonymous = True)

  positions = [[3.0, 1.5,0.0,0.0,0.0,0.0,0.0,0.0]]
  go('l', positions)
  torso_up()
  rospy.spin()
