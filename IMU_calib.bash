#!/bin/bash
SESSION=$USER

echo "Here we go for IMU measurements for calib"
sleep 2
tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:1 -n 'ROS'
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
tmux send-keys "roscore" C-m
tmux select-pane -t 1
tmux send-keys "sleep 5" C-m
tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
tmux send-keys "rosrun navio2_imu imu_magneto_calib 49" C-m

##LOGGING DATA
tmux split-window -v
tmux send-keys "sleep 10" C-m                                       #no needs to start earlier because 10/15 sec of calibration
tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
tmux send-keys "cd /home/pi/bagfiles" C-m
tmux send-keys "rosbag record -a" C-m


# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
