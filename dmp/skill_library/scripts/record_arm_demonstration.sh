#!/bin/bash

cd `rospack find skill_library`
mkdir -p demonstrations
cd demonstrations

rosbag record /joint_states /SL/right_arm_wrench_processed /SL/right_hand_strain_gauges -O $1
