To get inside docker container after ```docker pull codeinex/orb_img:ros_wrapper``` run ```sudo ./docker_run```.
Run ```./icuas_FBM1 <bagfile>.bag``` where <bagfile> is the name of the bagfile


What does the script do:-
    1. Creates a tmux session with multiple windows
    2. We run the following commands in each window:
     a. ```roscore```
     b. ```roslaunch orb_slam3_ros tum_rgbd.launch```
     c. ```rosbag play $1```, which takes the name of the bagfile from command line argument provided
     d. A python script ```initial_pose.py``` runs which subscribes to the topic ```/vicon_client/METRICS/pose``` to get the initial pose of the drone and saves it as ```initial_pose.txt```.
     e. When the rosbag is played entirely, we call ```rosservice call /orb_slam3/save_traj trajectory```. This saves the trajectory of the drone in camera frame as ```/root/.ros/trajectory_cam_traj.txt```.
     f. After this we run a python script ```transform.py```, which reads the trajectory generated and applies suitable transformations (including initial pose) and saves the resulting file as ```out.txt```, which is of the format ```timestamp, position(x, y, z), orientation(x, y, z, w)```.