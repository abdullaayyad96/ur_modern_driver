#!/usr/bin/env python

import geometry_msgs.msg as geo_msg
import rospy
import numpy as np
import tf
import tf2_ros
from scipy.spatial.transform import Rotation


if __name__ == '__main__':
	pub = rospy.Publisher('UR_driver_pose', geo_msg.PoseWithCovarianceStamped, queue_size=10)
	rospy.init_node('ur)pose_publisher', anonymous=True)
	listener = tf.TransformListener()
	
	rate = rospy.Rate(10)

	initial_pose = geo_msg.PoseWithCovarianceStamped()
	

	#get transform from world to camera at initialization
	while not rospy.is_shutdown():

		rate.sleep()
		try: 	
			(map_to_pose, map_to_pose_quat) = listener.lookupTransform("/base_link", "/ee_link", rospy.Time(0))		
			print("found")

			initial_pose.header.seq = 1
			initial_pose.header.stamp = rospy.get_rostime()
			initial_pose.header.frame_id = "base_link"
			initial_pose.pose.pose.position.x = map_to_pose[0]
			initial_pose.pose.pose.position.y = map_to_pose[1]
			initial_pose.pose.pose.position.z = map_to_pose[2]
			initial_pose.pose.pose.orientation.x = map_to_pose_quat[0]
			initial_pose.pose.pose.orientation.y = map_to_pose_quat[1]
			initial_pose.pose.pose.orientation.z = map_to_pose_quat[2]
			initial_pose.pose.pose.orientation.w = map_to_pose_quat[3]
			cov_matrix = 0.1 * np.eye(6)
			initial_pose.pose.covariance = cov_matrix.flatten().tolist()

			pub.publish(initial_pose)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print('transformation from world to d435_dummy not found')
			exit()




		
