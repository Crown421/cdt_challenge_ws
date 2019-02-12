#!/bin/sh

cd cdt_challenge_ws/
catkin build  --cmake-args -DCMAKE_BUILD_TYPE=Release
pwd
cd ..
pwd

. ~/.bashrc

