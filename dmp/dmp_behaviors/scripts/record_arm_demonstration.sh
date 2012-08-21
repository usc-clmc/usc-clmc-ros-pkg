#!/bin/bash

cd `rospack find arm_dmp_data`
mkdir -p dmp_data/demonstrations
cd dmp_data/demonstrations

rosbag record /joint_states \
 /SL/r_hand_wrench \
 /SL/l_hand_wrench \
 /SL/r_hand_strain_gauges \
 /SL/l_hand_strain_gauges \
 /SL/r_hand_accelerations \
 /SL/l_hand_accelerations \
 -O $1
