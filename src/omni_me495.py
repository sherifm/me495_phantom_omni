#!/usr/bin/env python  
import roslib
roslib.load_manifest('me495_phantom_omni')
import rospy
import math
import tf
import geometry_msgs.msg

if __name__=='__main__':
	rospy.init_node('mini_omni')

	listener = tf.TransformListener()

	Tsb = rospy.Publisher('stylus_to_base_transf',geometry_msgs.msg.Twist)

	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('base','stylus',rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
			
        S = geometry_msgs.msg.Twist()
     	S=(trans,rot) 
        Tsb.publish(S)

    	rate.sleep()






