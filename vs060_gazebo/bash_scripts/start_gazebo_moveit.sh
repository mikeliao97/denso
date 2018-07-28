#!/bin/bash

#Start Gazebo move it

echo "---Killing Gazebo + ROSCore---"

killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
killall -9 rosmaster & killall -9 roscore


echo "-----Starting up Gazebo+Denso Controllers ---"
tmux new-session 
tmux new-window "roslaunch vs060_gazebo vs060_gazebo_new.launch"

sleep 3 

tmux split-window "roslaunch vs060_newmoveit_config myrobot_planning_execution.launch"

echo "---Sleep for 3 seconds ---"

echo "---Starting Moveit!---"
