#!/usr/bin/env python

import sys
sys.path.append("/root/yolov7")
import rospy
from std_msgs.msg import String
import tf
import math
import numpy as np
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint, MultiDOFJointTrajectory
from geometry_msgs.msg import Twist, Transform, PoseStamped, Point, Quaternion
from quadrotor_msgs.msg import PositionCommand
from icuas23_competition.msg import poi
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu, Image
import numpy as np
import time
from detect_class import mlcv
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge
# OpenCV2 for saving an image
import cv2
bridge = CvBridge()
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg import Int32


# TODO: 
# 1. exploration Z points in obstacle or not check via cpp functions in ego planner -> WRAP the package -> Mahesh
# 2. z issue in min_Z_thresh for poi
# 3. send poi as soon as last explore point reached
# 4. Transition of yaw from -pi to pi while rotation
# 5. Similar control structure for Yaw before


class MLCV:

    def __init__(self):
        self.list = []
        self.coord_lst = []
        self.img=[]
        self.mlcv = mlcv()
        time.sleep(15)
        rospy.Subscriber("/red/camera/color/image_raw", Image, self.image_callback)
        self.crack_publisher = rospy.Publisher("/red/crack_image_annotated", Image, queue_size = 10)
        self.rate = rospy.Rate(100)
        self.flag = False



    def image_callback(self, msg):
        self.cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.list != []:
            #print ("detected")
            # print (self.coord_lst,self.list)
            imgMsg = bridge.cv2_to_imgmsg(self.img)
            self.crack_publisher.publish(imgMsg)
                #time.sleep(1)
            #print(np.shape(self.cv2_img))
            #self.rate.sleep()


class CONTROLLER:

	def __init__(self):
		# self.MINIMUM_Z_POI = 0.75
		self.MIN_Z_POI_THRESH = 0.9
		self.EXPLORE_TRAJ_ERROR_THRESH_z = 0.1
		self.EXPLORE_TRAJ_ERROR_THRESH_yaw = 0.15
		self.ANGULAR_VELOCITY = 0
		self.MAX_ZD = 4
		self.goal_point = Point()
		self.next_point = Point()
		self.prev_goal = Point()
		self.point_list_unchanged=[]
		self.position = [0, 0, 2]
		self.velocity = [0, 0, 0]
		self.acceleration = [0, 0, 0]
		self.yaw = 0
		self.yawdot = 0 # Radians
		self.camera_pose = PoseStamped()
		self.poi_array = poi()
		self.point_list = []  ##[Point(10,2,2)]
		self.current_pos = Point(1, 43, 2)
		# self.current_pos.x = 0
		# self.current_pos.y = 0
		# self.current_pos.z = 0
		self.current_orientation = Quaternion(0,0,0,0)
		self.traj = MultiDOFJointTrajectory()
		self.exit=False
		self.pxToMetre = 1 #0.72
		self.focalx = 381.36246688113556
		self.focaly = 381.36246688113556
		self.cy = 240.5
		self.cx = 320.5

		self.trajectory_publisher = rospy.Publisher('/red/position_hold/trajectory',MultiDOFJointTrajectoryPoint,queue_size=10)
		self.camera_pose_publisher = rospy.Publisher("/red/camera_pose", PoseStamped, queue_size = 10)
		rospy.Subscriber("/planning/pos_cmd", PositionCommand , self.poscmd_callback)
		rospy.Subscriber("/red/pose", PoseStamped, self.callback_pose)
		rospy.Subscriber("/waypoint_generator/waypoints", Path, self.callback_goal)

		######################################
		rospy.Subscriber("/red/camera/depth/image_raw",Image,self.depth_callback)
		######################################

		rospy.Subscriber("/red/poi", poi , self.callback_poi)
		self.position_publisher = rospy.Publisher('/red/tracker/input_trajectory',MultiDOFJointTrajectory,queue_size=30)
		self.waypt_pub = rospy.Publisher("/waypoint_generator/waypoints", Path, queue_size = 50)
		
		rospy.Subscriber("/red/odometry", Odometry , self.callback_odom)
		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=100)
		
		self.rate = rospy.Rate(100)


		#######################
		self.poi_notify_pub = rospy.Publisher('/is_poi', Int32, queue_size=30)
		self.is_poi = 0
		rospy.Subscriber('/resolve_issue', Int32, self.direct_pub_callback)
		self.direct_pub = 0
		#######################
		self.return_flag = False
		self.start = time.time()
		self.detection = MLCV()
		self.return_flag = False
		
		rospy.loginfo("Trajectory Node Initialized")
		rospy.sleep(0.5)

	def direct_pub_callback(self, msg):	
		self.direct_pub = msg.data


	def callback_goal(self, data):
		print("---goalset")
		if self.direct_pub == 1:
			self.next_point.x = data.poses[0].pose.position.x
			self.next_point.y = data.poses[0].pose.position.y
			self.next_point.z = data.poses[0].pose.position.z
		# self.prev_goal = goal_point
		else:
			self.goal_point.z = data.poses[0].pose.position.z
			self.goal_point.x = data.poses[0].pose.position.x
			self.goal_point.y = data.poses[0].pose.position.y

	def depth_callback(self,msg):
		depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		self.depth_array = np.array(depth_image, dtype=np.float32)
		# print (self.depth_array.shape)
	def poscmd_callback(self, data):
		self.position[0] = data.position.x
		self.position[1] = data.position.y
		self.position[2] = data.position.z

		self.velocity[0] = data.velocity.x
		self.velocity[1] = data.velocity.y
		self.velocity[2] = data.velocity.z

		self.acceleration[0] = data.acceleration.x
		self.acceleration[1] = data.acceleration.y
		self.acceleration[2] = data.acceleration.z
		
		self.yaw = data.yaw # in radians
		self.yawdot = data.yaw_dot

	def callback_odom(self, msg):
		self.current_pos.x = msg.pose.pose.position.x
		self.current_pos.y = msg.pose.pose.position.y
		self.current_pos.z = msg.pose.pose.position.z
		self.current_orientation.x = msg.pose.pose.orientation.x
		self.current_orientation.y = msg.pose.pose.orientation.y
		self.current_orientation.w = msg.pose.pose.orientation.w
		self.current_orientation.z = msg.pose.pose.orientation.z
		self.current_pitch, self.current_roll, self.current_yaw = self.euler_angles(self.current_orientation.x , self.current_orientation.y, self.current_orientation.z, self.current_orientation.w) # INVERTED
		# if(abs(self.current_yaw - np.pi*2) < self.EXPLORE_TRAJ_ERROR_THRESH_yaw):
		# 	self.yaw = self.yaw - np.pi*2
		# 	print("yaw GETS NEGETIVE", self.current_yaw)
		self.odom_pub.publish(msg)
		if(time.time() - self.start > 620 and not self.return_flag ):
			self.return_flag = True
			self.Return_to_takeoffpt()
	def Return_to_takeoffpt(self):
		pt = Path()
		pt.poses.append(PoseStamped())
		pt.poses[0].pose.position.x = self.takeoff_pt.x
		pt.poses[0].pose.position.y = self.takeoff_pt.y
		pt.poses[0].pose.position.z = self.takeoff_pt.z
		self.waypt_pub.publish(pt)
		self.goal_point.x = self.takeoff_pt.x
		self.goal_point.y = self.takeoff_pt.y
		self.goal_point.z = self.takeoff_pt.z
		abs_diff = np.linalg.norm(np.array((self.current_pos.x - self.goal_point.x, self.current_pos.y - self.goal_point.y, self.current_pos.z - self.goal_point.z)))
		while (abs_diff > 0.1):
			abs_diff = np.linalg.norm(np.array((self.current_pos.x - self.goal_point.x, self.current_pos.y - self.goal_point.y, self.current_pos.z - self.goal_point.z)))
			self.publish_waypoint()
			self.detection.list,self.detection.coord_lst,self.detection.img=self.detection.mlcv.detect(self.detection.cv2_img)
		print("Reached Takeoff point")
		while True:
			continue


	def euler_angles(self, x, y, z, w):
		# x, y, z, w = self.current_orientation.x , self.current_orientation.y, self.current_orientation.z, self.current_orientation.w
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll = math.atan2(t0, t1)
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = math.asin(t2)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw = math.atan2(t3, t4)
		return roll, pitch, yaw

	def callback_pose(self, data):
		self.camera_pose = data
		self.camera_pose.pose.position.x += 0.2
		self.camera_pose.pose.position.z += 0.05
		self.camera_pose_publisher.publish(self.camera_pose)


	def callback_poi(self, data):
		rospy.sleep(0.1)
		poi_no = []
		poi_list = []
		
		####################################################################
		no_of_poi = len(data.poi) 
		start_point = Point(self.current_pos.x, self.current_pos.y, self.current_pos.z)
		####################################################################

		print(no_of_poi)
		for i in range(0, no_of_poi): 
			if(data.poi[i].z < self.MIN_Z_POI_THRESH):
				data.poi[i].z = self.MIN_Z_POI_THRESH
			tmp = np.sqrt((data.poi[i].x - start_point.x)**2 + (data.poi[i].y - start_point.y)**2 + (data.poi[i].z - start_point.z)**2)
			poi_list.append((data.poi[i], tmp))
			poi_no.append(tmp)
		poi_no.sort()
		for i in range(0, no_of_poi):
			for j in range(0, no_of_poi):
				if poi_no[i] == poi_list[j][1]:
					self.point_list.append(poi_list[j][0])
		self.takeoff_pt = Point(self.current_pos.x, self.current_pos.y, self.current_pos.z)
		self.point_list_unchanged=self.point_list.copy()
		self.point_list.append(self.takeoff_pt)


	def print_poi(self):
		for i in range(0, len(self.point_list)):
			print("point", self.point_list[i].x, self.point_list[i].y, self.point_list[i].z)


	def get_n_pub_3d_input(self):
		x = float(input("Input x: "))     ###########################
		y = float(input("Input y: "))		#EDIT HERE TO TAKE INPUTS FROM TERMINAL
		z = float(input("Input z: "))		###########################

		pt = Path()
		pt_toSend = Point()
		pt_toSend.x = x
		pt_toSend.y = y
		pt_toSend.z = z

		pt.poses.append(PoseStamped())
		pt.poses[0].pose.position.x = x
		pt.poses[0].pose.position.y = y
		pt.poses[0].pose.position.z = z

		transforms = Transform()
		velocities = Twist()
		accelerations = Twist()

		transforms.translation.x = self.current_pos.x
		transforms.translation.y = self.current_pos.y
		transforms.translation.z = self.current_pos.z

		yaw = np.arctan2(pt_toSend.y - self.current_pos.y, pt_toSend.x - self.current_pos.x)
		quaternion = tf.transformations.quaternion_from_euler(yaw, 0, 0, axes = 'rzyx')
		transforms.rotation.x = quaternion[0]
		transforms.rotation.y = quaternion[1]
		transforms.rotation.z = quaternion[2]
		transforms.rotation.w = quaternion[3]

		point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Duration(1))
		self.trajectory_publisher.publish(point)

		start_time = time.time()

		while(time.time() - start_time < 2):
			pass
		self.waypt_pub.publish(pt)
		return pt_toSend	

	def yaw_before(self, i):
		transforms = Transform()
		velocities = Twist()
		accelerations = Twist()

		transforms.translation.x = self.current_pos.x
		transforms.translation.y = self.current_pos.y
		transforms.translation.z = self.current_pos.z

		# print(way_pt.poses.pose.position.y)

		yaw = np.arctan2(self.point_list[i].y - self.current_pos.y, self.point_list[i].x - self.current_pos.x)
		quaternion = tf.transformations.quaternion_from_euler(yaw, 0, 0, axes = 'rzyx') # DOUBT?
		# print("Quaternion",quat)
		transforms.rotation.x = quaternion[0]
		transforms.rotation.y = quaternion[1]
		transforms.rotation.z = quaternion[2]
		transforms.rotation.w = quaternion[3]

		point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Duration(1))

		self.trajectory_publisher.publish(point)
		while(abs(yaw) >= 0.1):
			# print(abs(yaw))
			yaw = np.arctan2(self.point_list[i].y - self.current_pos.y, self.point_list[i].x - self.current_pos.x) - self.current_yaw
			# self.rate.sleep()
			# print("in loop")

	def wait(self):
		abs_diff = np.linalg.norm(np.array((self.current_pos.x - self.next_point.x, self.current_pos.y - self.next_point.y, self.current_pos.z - self.next_point.z)))
		while (abs_diff > 0.1):
			abs_diff =np.linalg.norm(np.array((self.current_pos.x - self.next_point.x, self.current_pos.y - self.next_point.y, self.current_pos.z - self.next_point.z)))
			self.publish_waypoint()
			self.detection.list,self.detection.coord_lst,self.detection.img=self.detection.mlcv.detect(self.detection.cv2_img)
			self.delete_poi(in_explore=True,current_poi=self.curr_poi)
		pt1 = Path()

		pt1.poses.append(PoseStamped())
		pt1.poses[0].pose.position.x = self.goal_point.x
		pt1.poses[0].pose.position.y = self.goal_point.y
		pt1.poses[0].pose.position.z = self.goal_point.z

		rospy.sleep(1)
		self.waypt_pub.publish(pt1)
		
	def detect(self,points2):
		
		print('Detection Started')
		#########################################################  CHECK IF CURRENT Z IS FEASIBLE FROM EGO PLANNER
		if points2[2]<= 2:
			zd = 0
		elif points2[2]<=3 and points2[2]>2:
			zd = -1
		else :
			zd = -2
		print("Zd value:", zd)
		#########################################################

		for z in range(zd,self.MAX_ZD): #MAX_ZD is 5
			if self.exit:
				print ('exiting exploration')
				break
			self.Mov_Z_Dir(points2, z)
			rospy.sleep(0.1)
			self.Yaw_Traj_Gen(points2)
			rospy.sleep(0.1)
		self.exit=False

	def Mov_Z_Dir(self,pt,zd):
		print("Z Traj")
		print("-----------")
		quaternion = tf.transformations.quaternion_from_euler(-179, 0, 0, axes = 'rzyx')
		Next_wayp = Path()
		Next_wayp.poses.append(PoseStamped())
		Next_wayp.poses[0].pose.position.x = pt[0]
		Next_wayp.poses[0].pose.position.y = pt[1]
		Next_wayp.poses[0].pose.position.z = pt[2] + zd
		Next_wayp.poses[0].pose.orientation.x = quaternion[0]  #Try karke dekhna ta 
		Next_wayp.poses[0].pose.orientation.x = quaternion[1]
		Next_wayp.poses[0].pose.orientation.x = quaternion[2]
		Next_wayp.poses[0].pose.orientation.x = quaternion[3]
		self.waypt_pub.publish(Next_wayp)
		rospy.sleep(0.25)
		# self.waypt_pub.publish(Next_wayp)
		err = abs(self.current_pos.z - (self.goal_point.z))
		while (err>0.1):
			if self.direct_pub == 1:
				self.wait()
			err = abs(self.current_pos.z - (self.goal_point.z))
			self.publish_waypoint()
			self.detection.list,self.detection.coord_lst,self.detection.img=self.detection.mlcv.detect(self.detection.cv2_img)
			self.delete_poi(in_explore=True,current_poi=self.curr_poi)
			

		self.rate.sleep()

	def Yaw_Traj_Gen(self, pt, desired_yaw):  #Changed (added desired yaw)
		Kp = 0.15
		Kd = 0.10
		print("Yaw Traj")
		print("-----------")
		self.yaw_traj = MultiDOFJointTrajectory()

		transforms = Transform()
		velocities = Twist()
		accelerations = Twist()

		x_const = pt[0]
		y_const = pt[1]
		z_const = self.current_pos.z

		transforms.translation.x = pt[0]
		transforms.translation.y = pt[1]
		transforms.translation.z = self.current_pos.z

		# yaw = np.array([-175, -75, 25, 125, 160]) * np.pi/180
		yaw = np.array([desired_yaw])

		# ang_vel = [0,0.1,1] + [1 for i in range(len(yaw) - 6)] +[1,0.05,0]
		# ang_vel = [0,0.15,0.15,0.15,0]

		# ang_accel = [0,0.1,0.1] + [0 for i in range(len(yaw) - 6)] + [0.1,0.1,0]
		# ang_accel = [0, 0.1, 0.1, 0.1, 0]
		err_yaw = 0
		prev_err_yaw = 0
		prev_err_x = 0
		prev_err_y = 0
		ang_vel = 0
		ang_accel = 0
		a_vel = 0
		a_acc = 0

		def yaw_buffer(angle_err):
			if(angle_err > np.pi):
				angle_err -= 2*np.pi
			elif(angle_err < -np.pi):
				angle_err += 2*np.pi
			return angle_err


		for i in range(len(yaw)):
			##################################
			# err_yaw = 0
			ang_vel = Kp*err_yaw
			ang_accel = Kp/10*err_yaw
			#################################

			# quaternion = tf.transformations.quaternion_from_euler(self.convert_degree_to_rad(yaw[i]), 0, 0, axes = 'rzyx') # DOUBT?
			quaternion = self.quaternion_from_euler(self.current_pitch,self.current_roll,yaw[i])
			# print("Quaternion",quat)
			transforms.rotation.x = quaternion[0]
			transforms.rotation.y = quaternion[1]
			transforms.rotation.z = quaternion[2]
			transforms.rotation.w = quaternion[3]
			# velocities.angular.z = ang_vel[i]
			# accelerations.angular.z = ang_accel[i]

			######
			velocities.angular.z = ang_vel
			accelerations.angular.z = ang_accel


			point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Duration(1))
			self.yaw_traj.points.append(point)
			self.trajectory_publisher.publish(point)
			self.rate.sleep()
			err_yaw = yaw[i] - self.current_yaw
			err_x = self.current_pos.x - x_const
			err_y = self.current_pos.y - y_const
			# start = time.time()
			yaw_thresh = 0
			if( i < len(yaw) - 1 ):
				yaw_thresh = np.pi/12
			else:
				yaw_thresh = np.pi/12

			while abs(err_yaw) > yaw_thresh or abs(err_x) >0.15 or abs(err_y)>0.15:
				
				if self.direct_pub == 1:
					self.wait()

				err_yaw = yaw_buffer(yaw[i] - self.current_yaw)
				# if(abs(err_yaw) < np.pi/12 and i<len(yaw)):
				# 	i+=1
				# 	continue
				err_x = x_const - self.current_pos.x
				err_y = y_const - self.current_pos.y
				
				######################################
				ang_vel = Kp/2*err_yaw + Kd/2*(err_yaw - prev_err_yaw)
				ang_accel = Kp/10*err_yaw
				velocities.angular.z = ang_vel
				accelerations.angular.z = ang_accel
				velocities.linear.x = Kp/5*err_x + Kd/5*(err_x - prev_err_x)
				velocities.linear.y = Kp/5*err_y + Kd/5*(err_y - prev_err_y)

				transforms.translation.x = pt[0] #+ Kp/50*err_x + Kd/50*(err_x - prev_err_x)
				transforms.translation.y = pt[1] #+ Kp/50*err_y + Kd/50*(err_y - prev_err_y)
				# transforms.translation.z = self.current_pos.z

				point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Duration(1))
				self.trajectory_publisher.publish(point)
				self.rate.sleep()

				prev_err_yaw = err_yaw
				prev_err_x = err_x
				prev_err_y = err_y
				self.detection.list,self.detection.coord_lst,self.detection.img=self.detection.mlcv.detect(self.detection.cv2_img)
				self.delete_poi(in_explore=True,current_poi=self.curr_poi)
			if self.exit:
				break
           		# if self.list != []:
           		# 	print ("detected")
		# self.yaw_traj.header.stamp = rospy.Time.now()
		# self.yaw_traj.header.frame_id = 'frame'
		# self.yaw_traj.joint_names.append('base_link')
		# self.position_publisher.publish(self.yaw_traj)
		# self.rate.sleep()
		# err_yaw = abs(abs(self.current_yaw) - abs(self.convert_degree_to_rad(yaw[len(yaw)-1])))
		# while err_yaw > 0.1:
		# 	print("Error:",err_yaw)
		# 	print("yaw:", self.current_yaw)
		# 	err_yaw = abs(abs(self.current_yaw) - abs(self.convert_degree_to_rad(yaw[len(yaw)-1])))
		# rospy.sleep(5)

	
	def detect2(self,poi):
		
		z_high = poi[2] + self.MAX_ZD

		if poi[2] <= 2:
			z_low = poi[2]
		elif poi[2] <= 3 and poi[2] > 2:
			z_low = poi[2] - 1
		else :
			z_low = poi[2] - 2

		# explore_yaw = self.current_yaw
		# self.mov_z(self.current_yaw, z_low, poi)
		# self.Yaw_Traj_Gen(poi, self.current_yaw - np.pi/2)
		# self.mov_z(self.current_yaw, z_high, poi)
		# self.Yaw_Traj_Gen(poi, self.current_yaw - np.pi/2)
		# self.mov_z(self.current_yaw, z_low, poi)
		# self.Yaw_Traj_Gen(poi, self.current_yaw - np.pi/2)
		# self.mov_z(self.current_yaw, z_high, poi)
		# self.Yaw_Traj_Gen(poi, self.current_yaw - np.pi/2)
		# self.mov_z(self.current_yaw, z_low, poi)

		explore_yaw = self.current_yaw
		if self.exit:
			print ('exiting exploration')
			self.exit = False
			return

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))
		
		self.mov_z(explore_yaw, z_low, poi)
		explore_yaw -= np.pi/2
		
		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))
		
		self.Yaw_Traj_Gen(poi, explore_yaw)
		
		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))
		
		if self.exit:
			print ('exiting exploration')
			self.exit = False
			return
		self.mov_z(explore_yaw, z_high, poi)
		explore_yaw -= np.pi/2

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))

		self.Yaw_Traj_Gen(poi, explore_yaw)

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))


		if self.exit:
			print ('exiting exploration')
			self.exit = False
			return
		self.mov_z(self.current_yaw, z_low, poi)

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))

		explore_yaw -= np.pi/2
		self.Yaw_Traj_Gen(poi, explore_yaw)

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))


		if self.exit:
			print ('exiting exploration')
			self.exit = False
			return
		self.mov_z(self.current_yaw, z_high, poi)

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))

		explore_yaw -= np.pi/2
		self.Yaw_Traj_Gen(poi, self.current_yaw - np.pi/2)

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))

		if self.exit:
			print ('exiting exploration')
			self.exit = False
			return
		self.mov_z(self.current_yaw, z_low, poi)

		print("Explore yaw ", explore_yaw*(180/(np.pi)))
		print("Current Yaw ", self.current_yaw*(180/(np.pi)))
		self.exit = False

	def mov_z(self, explore_yaw, z_val, poi):
		wayp = Path()
		quaternion = tf.transformations.quaternion_from_euler(explore_yaw, 0, 0, axes = 'rzyx')
		wayp.poses.append(PoseStamped())
		wayp.poses[0].pose.position.x = poi[0]
		wayp.poses[0].pose.position.y = poi[1]
		wayp.poses[0].pose.position.z = z_val
		wayp.poses[0].pose.orientation.x = quaternion[0] 
		wayp.poses[0].pose.orientation.y = quaternion[1]
		wayp.poses[0].pose.orientation.z = quaternion[2]
		wayp.poses[0].pose.orientation.w = quaternion[3]
		self.waypt_pub.publish(wayp)
		rospy.sleep(0.25)
		err = abs(self.current_pos.z - self.goal_point.z)
		while(err > 0.2):
			if self.direct_pub == 1:
				self.wait()
			err = abs(self.current_pos.z - self.goal_point.z)
			self.publish_waypoint()
			self.detection.list,self.detection.coord_lst,self.detection.img=self.detection.mlcv.detect(self.detection.cv2_img)
			self.delete_poi(in_explore=True,current_poi=self.curr_poi)

		self.rate.sleep()



	def convert_rad_to_degree(self, angle):
		angle_deg = angle * 180 / np.pi
		return angle_deg

	def convert_degree_to_rad(self, angle_deg):

		#self.angular_speed_r = speed_deg * 3.14 / 180
		self.angle_r = angle_deg * np.pi / 180
		return self.angle_r

	def quaternion_from_euler(self,roll,pitch,yaw):
		qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
		qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
		qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
		return [qx, qy, qz, qw]

	def publish_waypoint(self):
		transforms = Transform()
		velocities = Twist()
		accelerations = Twist()

		transforms.translation.x = self.position[0]
		transforms.translation.y = self.position[1]
		transforms.translation.z = self.position[2]
		# print("yaw", yaw)
		quaternion = tf.transformations.quaternion_from_euler(self.yaw, 0, 0, axes = 'rzyx')
		# print("Quaternion",quat)
		transforms.rotation.x = quaternion[0]
		transforms.rotation.y = quaternion[1]
		transforms.rotation.z = quaternion[2]
		transforms.rotation.w = quaternion[3]
		if (self.is_poi == 0):
			velocities.angular.z = 2
		else:
			velocities.angular.z = self.yawdot

		velocities.linear.x = self.velocity[0]
		velocities.linear.y = self.velocity[1]
		velocities.linear.z = self.velocity[2]

		accelerations.linear.x = self.acceleration[0]
		accelerations.linear.y = self.acceleration[1]
		accelerations.linear.z = self.acceleration[2]

		point = MultiDOFJointTrajectoryPoint([transforms], [velocities], [accelerations], rospy.Duration(1))

		self.trajectory_publisher.publish(point)
		self.rate.sleep()

	def quaternion_rotation_matrix(self,q0,q1,q2,q3):

		# First row of the rotation matrix
		r00 = 2 * (q0 * q0 + q1 * q1) - 1
		r01 = 2 * (q1 * q2 - q0 * q3)
		r02 = 2 * (q1 * q3 + q0 * q2)

		# Second row of the rotation matrix
		r10 = 2 * (q1 * q2 + q0 * q3)
		r11 = 2 * (q0 * q0 + q2 * q2) - 1
		r12 = 2 * (q2 * q3 - q0 * q1)

		# Third row of the rotation matrix
		r20 = 2 * (q1 * q3 - q0 * q2)
		r21 = 2 * (q2 * q3 + q0 * q1)
		r22 = 2 * (q0 * q0 + q3 * q3) - 1

		# 3x3 rotation matrix
		rot_matrix = np.array([[r00, r01, r02],
		                        [r10, r11, r12],
		                        [r20, r21, r22]])

		return rot_matrix

	def transform(self):
		lst=[]
		coord_lst=self.detection.coord_lst
		for i in coord_lst:
			x1,y1,x2,y2=i
			# print (x1,x2,y1,y2)
			x_mid=(x1+x2)/2
			y_mid=(y1+y2)/2
			depth=0
			for k in range (10):
				x=np.random.randint(low=x1,high=x2-1)
				y=np.random.randint(low=y1,high=y2-1)
				try:
					depth += self.depth_array[y,x]
				except:
					pass
			depth=depth/10
			#imgMsg = bridge.cv2_to_imgmsg(img)
			#self.crack_publisher.publish(imgMsgx`)

			self.loc = self.convert_from_uvd(x_mid,y_mid,depth)
			self.loc = np.reshape(self.loc,(3,1))
			x=self.loc[0]
			y=self.loc[1]

			x_pose=self.camera_pose.pose.position.x
			y_pose=self.camera_pose.pose.position.y
			z_pose=self.camera_pose.pose.position.z
			self.camera_pose_arr=np.array([x_pose,y_pose,z_pose]).T
			self.rot_matrix = self.quaternion_rotation_matrix(self.current_orientation.x, self.current_orientation.y, self.current_orientation.z, self.current_orientation.w)
			self.rot_matrix = np.linalg.inv(self.rot_matrix)
			self.mat = np.matmul(self.rot_matrix,self.loc)
			self.camera_pose_arr = np.reshape(self.camera_pose_arr,(3,1))
			self.mat = np.add(self.mat,self.camera_pose_arr)
			# print (self.mat)
			lst.append(self.mat)
			# print("global frame:",self.mat)
		return lst


	def convert_from_uvd(self, u, v,d):
		d *= self.pxToMetre
		x_over_z = (u-self.cx ) / self.focalx
		y_over_z = (v-self.cy ) / self.focaly
		#z = d / np.sqrt(1. + x_over_z*2 + y_over_z*2)
		z=d
		x = x_over_z * z
		y = y_over_z * z
		arr = np.array((z,-x,-y))
		return arr
	def delete_poi(self,in_explore=False,current_poi=None):
		if(len(self.detection.coord_lst) > 0):
			bounding_boxes = self.transform()
			for box in bounding_boxes:
				x,y,z=box
				min_dist=100
				min_dist1=100
				point=-1
				point2=-1
				# print (self.point_list)
				for i in range(len(self.point_list)):
					x1=self.point_list[i].x
					y1=self.point_list[i].y
					dist=((x-x1)**2 + (y-y1)**2)**0.5
					if dist<min_dist:
						min_dist=dist
						point=self.point_list[i]
				# print (self.point_list_unchanged)
				for i in range(len(self.point_list_unchanged)):
					x1=self.point_list_unchanged[i].x
					y1=self.point_list_unchanged[i].y
					dist=((x-x1)**2 + (y-y1)**2)**0.5
					# print (dist)
					if dist<min_dist1:
						min_dist1=dist
						point2=self.point_list_unchanged[i]
				if min_dist1==min_dist and min_dist<2.5 and point2.x==point.x and point2.y==point.y:

					self.point_list.remove(point)
					ind=self.points2.index(point)
					self.points2.remove(point)
					self.points.pop(ind)
					print ('deleting',point.x,point.y,point.z)
					print ('global coordinates',box)
					print ()
					if in_explore:
						if current_poi.x==point.x and current_poi.y==point.y:
							self.i=self.i-1  #indexing change
							self.exit=True
							break
				# else:
				# 	print (min_dist1,min_dist)
				# 	try:
				# 		print (point2.x,point.x)
				# 		print (point2.y,point.y)
				# 	except:
				# 		pass
			# self.detection.coord_lst=[]

def main_terminal():

	rospy.init_node('Trajectory_Publisher_Node', anonymous = True)
	kopterworx = CONTROLLER()
	kopterworx.is_poi=1
	kopterworx.poi_notify_pub.publish(kopterworx.is_poi)
	points2 =kopterworx.get_n_pub_3d_input()
	kopterworx.curr_poi=kopterworx.goal_point
	while not rospy.is_shutdown():
		abs_diff = np.array((kopterworx.current_pos.x - kopterworx.goal_point.x, kopterworx.current_pos.y - kopterworx.goal_point.y, kopterworx.current_pos.z - kopterworx.goal_point.z))
		print("Current_pos: ", kopterworx.current_pos.x, kopterworx.current_pos.y, kopterworx.current_pos.z)
		print("Points2: ", points2.x, points2.y, points2.z)
		rospy.loginfo(np.linalg.norm(abs_diff))
		if np.linalg.norm(abs_diff) <= 0.5:
			kopterworx.is_poi=0
			kopterworx.poi_notify_pub.publish(kopterworx.is_poi)
			kopterworx.detect2([kopterworx.goal_point.x, kopterworx.goal_point.y, kopterworx.goal_point.z])
			kopterworx.is_poi=1
			kopterworx.poi_notify_pub.publish(kopterworx.is_poi)
			points2 =kopterworx.get_n_pub_3d_input()
			
		while np.linalg.norm(abs_diff) >= 0.5:
			abs_diff = np.array((kopterworx.current_pos.x - kopterworx.goal_point.x, kopterworx.current_pos.y - kopterworx.goal_point.y, kopterworx.current_pos.z - kopterworx.goal_point.z))
			if kopterworx.direct_pub == 1:
				print("WAITING")
				kopterworx.wait()
			kopterworx.publish_waypoint()
			kopterworx.detection.list,kopterworx.detection.coord_lst,kopterworx.detection.img=kopterworx.detection.mlcv.detect(kopterworx.detection.cv2_img)
			# kopterworx.delete_poi(in_explore=False,current_poi=kopterworx.curr_poi)



def main_poi():
	rospy.init_node('Trajectory_Publisher_Node', anonymous = True)
	kopterworx = CONTROLLER()
	start = time.time()
	poi_dist = 3
	
	while(len(kopterworx.point_list) == 0):
		pass
	kopterworx.print_poi()	

	kopterworx.points = []
	kopterworx.points2 = kopterworx.point_list.copy()                  ############ REPLACE POINT2 ENTIRELY, PROFFESSIONALLY ~ SWADINE ############
	for i in range(len(kopterworx.point_list)):
		pt = Path()

		pt.poses.append(PoseStamped())
		pt.poses[0].pose.position.x = kopterworx.point_list[i].x
		pt.poses[0].pose.position.y = kopterworx.point_list[i].y
		pt.poses[0].pose.position.z = kopterworx.point_list[i].z

		rospy.sleep(1)
		kopterworx.points.append(pt)
	print("len", len(kopterworx.points))

	kopterworx.i = 0
	# # detection.flag = True
	#kopterworx.detect([kopterworx.current_pos.x, kopterworx.current_pos.y, kopterworx.current_pos.z])
	# # detection.flag = False
	kopterworx.yaw_before(kopterworx.i)
	############
	kopterworx.is_poi=1
	kopterworx.poi_notify_pub.publish(kopterworx.is_poi)
	############
	kopterworx.waypt_pub.publish(kopterworx.points[kopterworx.i])

	while not rospy.is_shutdown():
		kopterworx.curr_points2=kopterworx.points2
		kopterworx.curr_points=kopterworx.points
		kopterworx.curr_poi=kopterworx.points2[kopterworx.i]
		abs_diff = np.array((kopterworx.current_pos.x - kopterworx.points2[kopterworx.i].x, kopterworx.current_pos.y - kopterworx.points2[kopterworx.i].y, kopterworx.current_pos.z - kopterworx.points2[kopterworx.i].z))
		print("Current_pos: ", kopterworx.current_pos.x, kopterworx.current_pos.y, kopterworx.current_pos.z)
		print("kopterworx.Points2: ", kopterworx.points2[kopterworx.i].x, kopterworx.points2[kopterworx.i].y, kopterworx.points2[kopterworx.i].z)
		rospy.loginfo(np.linalg.norm(abs_diff))
		while np.linalg.norm(abs_diff) >= poi_dist:
			abs_diff = np.array((kopterworx.current_pos.x - kopterworx.goal_point.x, kopterworx.current_pos.y - kopterworx.goal_point.y, kopterworx.current_pos.z - kopterworx.goal_point.z))
			if kopterworx.direct_pub == 1:
				print("WAITING")
				kopterworx.wait()
			kopterworx.publish_waypoint()
			kopterworx.detection.list,kopterworx.detection.coord_lst,kopterworx.detection.img=kopterworx.detection.mlcv.detect(kopterworx.detection.cv2_img)
			kopterworx.delete_poi(in_explore=False,current_poi=kopterworx.curr_poi)

		if kopterworx.i == len(kopterworx.points)-1:		
			print("break", kopterworx.i)
			break
		if np.linalg.norm(abs_diff) <= poi_dist:
			if(kopterworx.i < len(kopterworx.points)-1):
				#######
				kopterworx.is_poi=0
				kopterworx.poi_notify_pub.publish(kopterworx.is_poi)
				#######
				#kopterworx.detection.flag = True
				# kopterworx.detect([kopterworx.points2[kopterworx.i].x, kopterworx.points2[kopterworx.i].y, kopterworx.points2[kopterworx.i].z])
				kopterworx.detect2([kopterworx.curr_points2[kopterworx.i].x, kopterworx.curr_points2[kopterworx.i].y, kopterworx.curr_points2[kopterworx.i].z])
				#kopterworx.detection.flag = False
			rospy.sleep(1)
			
			print("in loop", kopterworx.i)

			#########
			kopterworx.i+=1
			kopterworx.is_poi=1
			kopterworx.poi_notify_pub.publish(kopterworx.is_poi)
			#########
			kopterworx.yaw_before(kopterworx.i)
			kopterworx.waypt_pub.publish(kopterworx.curr_points[kopterworx.i])
			

	kopterworx.waypt_pub.publish(kopterworx.curr_points[len(kopterworx.points) - 1])
	# while np.linalg.norm(abs_diff) >= 0.5:
	# 		abs_diff = np.array((kopterworx.current_pos.x - kopterworx.points2[i].x, kopterworx.current_pos.y - kopterworx.points2[i].y, kopterworx.current_pos.z - kopterworx.points2[i].z))
	# 		kopterworx.publish_waypoint()
	# 		kopterworx.detection.list,kopterworx.detection.img=kopterworx.detection.mlcv.detect(kopterworx.detection.cv2_img)


if __name__ == '__main__':

	try:

		main_poi()

	except rospy.ROSInterruptException:

		print("ROS Terminated")
		pass