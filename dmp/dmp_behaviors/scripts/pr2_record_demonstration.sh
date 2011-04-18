#!/bin/bash

cd `rospack find skill_library`
mkdir -p demonstrations
cd demonstrations

rosbag record /joint_states -O $1
