#!/usr/bin/env python  
import roslib
roslib.load_manifest('me495_phantom_omni')
import rospy
import math
import tf
from geometry_msgs.msg import Twist,Vector3
from tf.transformations import euler_from_quaternion

if __name__=='__main__':
	rospy.init_node('mini_omni')

	listener = tf.TransformListener()

	Tsb = rospy.Publisher('stylus_to_base_transf',Twist)

	rate = rospy.Rate(5.0)
	while not rospy.is_shutdown():
		try:
			(trans,quat) = listener.lookupTransform('base','stylus',rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
			
		rot= euler_from_quaternion(quat)
		S=Twist(Vector3(trans[0],trans[1],trans[2]),(Vector3(rot[0],rot[1],rot[2])))
		Tsb.publish(S)
		rate.sleep()







