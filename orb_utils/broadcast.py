import rospy

import tf
from geometry_msgs.msg import PoseStamped, Transform, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster
import os.path
import time

class initial_pose():
    def __init__(self, node_name):  
        
        self.tf=None
        self.pub = StaticTransformBroadcaster()
        while self.tf==None:
            self.gt_sub = rospy.Subscriber('/falconblack/vrpn_client/estimated_odometry', Odometry, self.callback)
            rospy.sleep(1)    	
        print ('hello')    
        self.publish() 
        
        
        
      
      

    def callback(self, p):
        self.tf = TransformStamped()
        self.tf.header.stamp    = rospy.Time.now()
        
        self.tf.header.frame_id = 'gt'
        self.tf.child_frame_id  = 'skew'
        self.tf.transform = Transform()
        self.tf.transform.translation = p.pose.pose.position
        print (self.tf.transform.translation)
        self.tf.transform.rotation    = p.pose.pose.orientation
        print (self.tf.transform.rotation)
        self.pub.sendTransform(self.tf)
        self.gt_sub.unregister()
        
        




    def publish(self):
        while True:
            if self.tf!= None:
                vicon2init_pose_broadcaster = StaticTransformBroadcaster()
                vicon2init_pose = TransformStamped()    
                vicon2init_pose.header.stamp = rospy.Time.now()
                
                vicon2init_pose.header.frame_id = "gt"
                vicon2init_pose.child_frame_id = "skew"
                vicon2init_pose.transform.translation=self.tf.transform.translation
                vicon2init_pose.transform.rotation=self.tf.transform.rotation
                vicon2init_pose_broadcaster.sendTransform(vicon2init_pose)

if __name__=='__main__':
    rospy.init_node('hello', anonymous=False)
    a=initial_pose('hello')
