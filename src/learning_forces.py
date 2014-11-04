#!/usr/bin/env python  
import rospy
import math
import tf
from geometry_msgs.msg import Vector3
from phantom_omni.msg import OmniFeedback

if __name__=='__main__':
	rospy.init_node('learning_forces')

	print "test"
	pub = rospy.Publisher('/omni1_force_feedback',OmniFeedback)

	Fd = OmniFeedback(force=Vector3(0,0,0),position=Vector3(5,0,0))

	while not rospy.is_shutdown():
		try:
			pub.publish(Fd)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	