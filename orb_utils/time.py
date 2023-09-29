import rospy
rospy.init_node("Node", anonymous=True)
while True:
	print (rospy.Time.now())
	rospy.sleep(0.5)