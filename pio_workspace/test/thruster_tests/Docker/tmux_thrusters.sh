#!/bin/bash

# Name your tmux session
SESSION=auv_session

# Start a new tmux session, detached
tmux new-session -d -s $SESSION

# In the first pane
tmux send-keys -t $SESSION:0.0 'cd catkin_ws && catkin build && source devel/setup.bash && roslaunch propulsion propulsion.launch' C-m

# Split horizontally to make second pane
tmux split-window -h -t $SESSION

# In the second pane
tmux send-keys -t $SESSION:0.1 'cd catkin_ws && catkin build && source devel/setup.bash && rostopic echo /all/data' C-m

# Split the first pane vertically to make third pane
tmux select-pane -t $SESSION:0.0
tmux split-window -v -t $SESSION

# In the third pane
tmux send-keys -t $SESSION:0.2 'cd catkin_ws && catkin build && source devel/setup.bash && rostopic pub -r 10 /propulsion/microseconds auv_msgs/ThrusterMicroseconds "{microseconds: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]}"' C-m

# Attach to the tmux session
tmux attach-session -t $SESSION

