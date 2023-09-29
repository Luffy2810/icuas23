import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros as tf2

def write():
    # Save the pose information to a text file
    # with open('poses.txt', 'w') as file:
    #     file.write(str(data.pose) + '\n')
    global file
    global buff
    global tfl
    try:
        out = buff.lookup_transform(
            'gt', 'base_link', rospy.Time(0))
        t=out.header.stamp.to_sec()
        x=out.transform.translation.x 
        y=out.transform.translation.y
        z=out.transform.translation.z  
        rx=out.transform.rotation.x
        ry=out.transform.rotation.y
        rz=out.transform.rotation.z
        rw=out.transform.rotation.w
        file.write(str(t)+' '+str(x)+' '+str(y)+' '+str(z)+' '+'\n')
        print (t)

    except Exception as e: 
        rospy.logerr("Unable to get the tranform from {} to {}".format('vicon', 'base_link'))
        print (e)


def listener():
    global file
    file = open('pose.txt', 'w')
    while True:
        write()
        rospy.sleep(0.05)
if __name__ == '__main__':
    rospy.init_node('writer')
    buff = tf2.Buffer()
    tfl = tf2.TransformListener(buff)
    listener()
