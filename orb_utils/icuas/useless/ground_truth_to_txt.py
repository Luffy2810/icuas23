
#Ground Truth#
# FOR ORB use (afteer source) rosservice call /orb_slam3/save_traj abbc.txt after entire data is run
#DATA

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

f = open("bag3_gt.txt", "w")


def callback(data):
    t=data.header.stamp.to_sec()
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    f.write(str(float(t))+' '+str(float(x))+' '+str(float(y))+' '+str(float(z))+' '+"\n")


def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/falconblack/vrpn_client/estimated_odometry', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()


