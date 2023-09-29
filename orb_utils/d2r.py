import rospy
from sensor_msgs.msg import Image,CompressedImage
from sensor_msgs.msg import Imu
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import time
import cv2
import numpy as np
import time


class Nodo(object):
    def __init__(self):
        # Params
        self.depth = None
        self.color=None
        self.br = CvBridge()
        self.fx=391.7041931152344 
        self.cx=317.6928405761719
        self.fy=391.7041931152344
        self.cy=239.3055419921875

        self.fx_rgb=611.9695434570312
        self.cx_rgb=318.71343994140625
        self.fy_rgb=612.0466918945312
        self.cy_rgb=237.70240783691406

        extrinsics = {'rotation': np.array([ [0.9999583959579468, -0.006194998510181904, -0.006697379983961582,],[ 0.006227952428162098, 0.9999685287475586, 0.004910848569124937], [0.0066667464561760426, -0.004952355287969112, 0.9999655485153198]]) ,
                    'translation': np.array([0.014821390621364117, 0.0001570069434819743, 0.00037033073022030294])}
        self.R=extrinsics['rotation']
        self.T=extrinsics['translation']

        rospy.Subscriber("/red/camera/color/image_raw/compressed",CompressedImage,self.callback)
        rospy.Subscriber("/red/camera/depth/image_rect_raw",Image,self.callback1)
        # rospy.Subscriber("/dji_sdk/imu", Imu, self.imu_callback)
        self.pub_rgb = rospy.Publisher('image_rgb', Image, queue_size=100)
        self.pub_depth = rospy.Publisher('image_depth', Image, queue_size=100)
        self.pub_imu = rospy.Publisher('imu', Imu, queue_size=100)
        self.imu=None
        self.t=rospy.Time.now()
        while True:
            print ('in loop')
            if self.color is not None and self.depth is not None:
                time.sleep(1)
                break
                
        print ('Initialized')
        
    def callback(self, msg):
        self.color = self.br.compressed_imgmsg_to_cv2(msg)
        self.t=rospy.Time.now()
    def callback1(self, msg):
        self.depth = self.br.imgmsg_to_cv2(msg,'16UC1')
        self.depth = cv2.resize(self.depth, (640, 480),  interpolation = cv2.INTER_NEAREST) 
        self.depth_data = self.align_depth_to_rgb(self.depth) # Modify this function according to your source
        # self.depth_data=msg

    def imu_callback(self,msg):
        self.t=rospy.Time.now()
        self.imu=msg
        # print(msg)
        modified_imu = Imu()
        modified_imu.header = self.imu.header  # Copy the original header
        modified_imu.header.stamp = self.t # Assign a new timestamp
        modified_imu.angular_velocity = self.imu.angular_velocity
        modified_imu.linear_acceleration = self.imu.linear_acceleration

        # Publish the modified IMU data
        #print (modified_imu)
        self.pub_imu.publish(modified_imu)


    def align_depth_to_rgb(self,imgd):
        xx, yy = np.meshgrid(np.arange(640), np.arange(480))
        xx_r = (xx - self.cx) * imgd / self.fx
        yy_r = (yy - self.cy) * imgd / self.fy
        zz = imgd
        p3 = np.stack((xx_r, yy_r, zz), axis=-1)

        p3_ = (self.R @ p3.reshape(-1, 3).T + self.T.reshape(-1, 1)).T.reshape(p3.shape)

        p2d_x = (p3_[:, :, 0] * self.fx_rgb / p3_[:, :, 2]) + self.cx_rgb
        p2d_y = (p3_[:, :, 1] * self.fy_rgb / p3_[:, :, 2]) + self.cy_rgb

        depth_new = np.zeros((480, 640))

        mask = np.logical_and(np.logical_and(p2d_x >= 0, p2d_x < 640), np.logical_and(p2d_y >= 0, p2d_y < 480))

        p2d_x_int = p2d_x.astype(int)
        p2d_y_int = p2d_y.astype(int)

        depth_new[p2d_y_int[mask], p2d_x_int[mask]] = imgd[mask]

        return depth_new

    def start(self):
    # Align depth image to RGB image
        while True:
            a1=self.depth_data.copy()
            a2=self.color.copy()
            a1=a1*1.05
            # transformed_depth_image_points = self.align_depth_to_rgb(self.depth)
            # self.depth_data=a1.astype(np.uint16)
        # Create the Image message


            # depth_new_img1=np.zeros([480,640,3])
            # depth_new_img1[:,:,0]=self.depth_data
            # added_image = cv2.addWeighted(self.color.astype(int),0.4,depth_new_img1.astype(int),0.05,0)
            # added_image = added_image.astype(np.uint8)
            # transformed_depth_image_points=transformed_depth_image_points.astype(np.int16)
            # imgMsg = self.br.cv2_to_imgmsg(transformed_depth_image_points, "16SC1")
            # # imgMsg = self.br.cv2_to_imgmsg(self.depth, "16UC1")
            # self.pub.publish(imgMsg)
            # rospy.Rate(15.0).sleep()
            depth_msg = Image()
            
            depth_msg.header.stamp = self.t
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            depth_msg.height = self.depth_data.shape[0]
            depth_msg.width = self.depth_data.shape[1]
            a1=a1.astype(np.uint16)
            depth_msg.encoding = "16UC1"  # 32-bit floating-point depth values
            depth_msg.step = depth_msg.width * 2  # 4 bytes per depth value
            
            depth_msg.data = a1.tostring()
            self.pub_depth.publish(depth_msg)



            depth_msg_RGB = Image()
            depth_msg_RGB.header.stamp = self.t
            depth_msg_RGB.header.frame_id = "camera_color_optical_frame"
            depth_msg_RGB.height = self.color.shape[0]
            depth_msg_RGB.width = self.color.shape[1]
            depth_msg_RGB.encoding = "rgb8"  # 32-bit floating-point depth values
            depth_msg_RGB.step = depth_msg_RGB.width *3   # 4 bytes per depth value
            depth_msg_RGB.data = a2.tostring()
            self.pub_rgb.publish(depth_msg_RGB)


            rospy.Rate(30.0).sleep()

            # cv2.imshow('out', added_image)
            # cv2.waitKey(1)    
            # pass


if __name__ == '__main__':
    rospy.init_node("Node", anonymous=True)
    my_node = Nodo()
    my_node.start()
