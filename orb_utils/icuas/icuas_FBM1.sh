#tmux \
#    new-session 'roscore; bash' \; \
#    new-window 'roslaunch orb_slam3_ros tum_rgbd.launch; bash' \; \
#    new-window 'rosbag play $1; rosservice call /orb_slam3/save_traj trajectory; cp ~/.ros/trajectory_cam_traj.txt out.txt; bash'



#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Incorrect number of parameters. Usage:"
    echo "./icuas-FBM1.sh </path/to/bag> <result_filename>"
    exit -1
fi
echo "Evaluation bag path: $1"
echo "Result file name: $2"

result_path="/home/icuas/results/"$2
echo "Result file path: $result_path"

# Kill any previous session (-t -> target session, -a -> all other sessions )
tmux kill-session -t "SESSION"
tmux kill-session -a

# Create new session  (-2 allows 256 colors in the terminal, -s -> session name, -d -> not attach to>
tmux -2 new-session -d -s "SESSION"

# Create roscore 
# send-keys writes the string into the sesssion (-t -> target session , C-m -> press Enter Button)
tmux rename-window -t $SESSION:0 'roscore'
tmux send-keys -t $SESSION:0 "roscore" C-m

tmux new-window -t $SESSION:1 -n 'orb'
tmux send-keys -t $SESSION:1 "roslaunch orb_slam3_ros tum_rgbd.launch" C-m

sleep 3

tmux new-window -t $SESSION:2 -n 'rosbag play'
tmux send-keys -t $SESSION:2 "sleep 5 ;rosbag play $1; rosservice call /orb_slam3/save_traj trajectory; cp ~/.ros/trajectory_cam_traj.txt ~/icuas/result/$2" C-m

tmux attach-session -t $SESSION:2

echo "Done!"
