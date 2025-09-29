#!/bin/bash

# THIS IS THE SCRIPT THAT RUNS WHEN THIS DOCKER CONTAINER STARTS UP AND IT LAUNCHES THE MATE DESKTOP SESSION AND THE ROS NODES
set -e

source /opt/ros/kinetic/setup.bash
source /root/catkin_ws/install_isolated/setup.bash

service ssh start

# retry loop for mate-session
(
  while true; do
    echo "Starting mate-session..."
    mate-session
    echo "mate-session crashed, retrying..."
    sleep 0.5
  done
) &

# run roslaunch in foreground
exec roslaunch usv_sim sailboat_scenario3.launch parse:=false