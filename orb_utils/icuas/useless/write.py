import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    # Save the pose information to a text file
    # with open('poses.txt', 'w') as file:
    #     file.write(str(data.pose) + '\n')
    global file
    x=data.pose.position.x 
    y=data.pose.position.y 
    z=data.pose.position.z  
    qx=data.pose.orientation.x 
    qy=data.pose.orientation.y 
    qz=data.pose.orientation.z 
    qw=data.pose.orientation.w 
    file.write(str(x)+' '+str(y)+' '+str(z)+' '+str(qx)+' '+str(qy)+' '+str(qz)+' '+str(qw)+'\n')
    print ('writing')

def listener():
    rospy.init_node('pose_listener', anonymous=True)
    global file
    file = open('pose.txt', 'w')
    rospy.Subscriber('/odometry/final', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
