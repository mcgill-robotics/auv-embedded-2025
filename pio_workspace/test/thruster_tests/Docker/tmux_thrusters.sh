#!/bin/bash

# Name your tmux session
SESSION=auv_session

# Start a new tmux session, detached
tmux new-session -d -s $SESSION

# In the first pane: Start roscore
tmux send-keys -t $SESSION:0.0 'roscore' C-m

# Split horizontally to make second pane
tmux split-window -h -t $SESSION:0

# In the second pane: cd into catkin_ws, build, and launch the propulsion test
tmux send-keys -t $SESSION:0.1 'cd catkin_ws && catkin build && source devel/setup.bash && roslaunch propulsion thrustertest.launch' C-m

# Split the first pane vertically to make third pane
tmux select-pane -t $SESSION:0.0
tmux split-window -v -t $SESSION

# In the third pane: cd into catkin_ws, build, and echo /propulsion/microseconds topic
tmux send-keys -t $SESSION:0.2 'cd catkin_ws && catkin build && source devel/setup.bash && rostopic echo /propulsion/microseconds' C-m

# Create a new pane and run rosserial node on /dev/ttyACM1
tmux split-window -h -t $SESSION:0.2
tmux send-keys -t $SESSION:0.3 'roslaunch rosserial_python serial_node.launch dev:=/dev/ttyACM1' C-m

# Create another pane and echo /all/data
tmux split-window -v -t $SESSION:0.3
tmux send-keys -t $SESSION:0.4 'cd catkin_ws && catkin build && source devel/setup.bash && rostopic echo /all/data' C-m

# Attach to the tmux session
tmux attach-session -t $SESSION
