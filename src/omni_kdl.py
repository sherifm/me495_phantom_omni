#!/usr/bin/env python  
import rospy
import math
from urdf_parser_py.urdf import URDF
from geometry_msgs.msg import Twist,Vector3
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from sensor_msgs.msg import JointState
from pykdl_utils.kdl_kinematics import KDLKinematics
import numpy

def kdl_forward_kin (data):
	q = data.position
	pose_stylus = omni_kin.forward(q)
	q_ik = omni_kin.inverse(pose_stylus, numpy.array(q)+0.2)

	if q_ik is not None:
	 	pose_sol = omni_kin.forward(q_ik)
	print q_ik


if __name__=='__main__':
	rospy.init_node('omni_kdl')


	# rate = rospy.Rate(5.0)
	omni = URDF.from_parameter_server()
	tree = kdl_tree_from_urdf_model(omni)
	omni_kin = KDLKinematics(omni, "base", "stylus")

	rospy.Subscriber("omni1_joint_states", JointState, kdl_forward_kin)

	rospy.spin()

	# while not rospy.is_shutdown():
	# 	try:

	# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 		continue
		# rate.sleep()