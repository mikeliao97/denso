#!/bin/bash

#Start Gazebo move it

echo "---Killing Gazebo + ROSCore---"

killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient
killall -9 rosmaster & killall -9 roscore


echo "-----Starting up Gazebo+Denso Controllers ---"
session_name="gazebo"

#Remove any old tmux sessions  if there is one
tmux kill-session -t ${session_name}

#Start a new session
tmux new -d -s ${session_name}
tmux set-option remain-on-exit on

tmux selectp -t ${session_name}
tmux split-window -v "source setup && roslaunch vs060_gazebo vs060_gazebo_new.launch"

#Sleep for 3 seconds for gazebo controllers to come up
echo "---Sleep for 3 seconds ---"
sleep 3 

echo "---Starting Moveit!---"
tmux split-window -h "source setup && roslaunch vs060_newmoveit_config myrobot_planning_execution.launch"

tmux attach -t ${session_name}
