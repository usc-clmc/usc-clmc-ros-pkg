#!/bin/bash

cd `rospack find arm_dmp_data`
mkdir -p dmp_data/demonstrations
cd dmp_data/demonstrations

rosbag record /joint_states /SL/right_arm_wrench_processed /SL/right_hand_strain_gauges -O $1
