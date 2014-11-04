#!/usr/bin/env python 

"""
Nurullah Gulmus & Sherif Mostafa

This node is a demo for a mini project with the Phantom Omni.
ADD EXPLANATION FOR WHAT IT DOES WHEN FINISHED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

SUBSCRIBERS


"""

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist,Vector3
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

import numpy



class OmniMiniProj:

	def __init__(self):
		
		#tf listeners and publishers
		self.tfListener = tf.TransformListener() #tf.TransfromListener is a subclass that subscribes to the "/tf" message topic and adds 
		#transform data to the tf data structure

		self.Tsb = rospy.Publisher('stylus_to_base_transf',Twist) #Create a publisher to a new topic name calle "stylus_to_base_transf with 
		#a message type Twist"

		#Find the omni from parameter server
		self.omni = URDF.from_parameter_server() #A node that initializes and runs the Phantom Omni has to be running in the background for 
		#from_parameter_server to to find the parameter

		#kdl subscribers and publishers
		self.join_state_sub = rospy.Subscriber("omni1_joint_states", JointState, self.get_joint_states)

		#OmniMiniProjTimers
		self.timer1 = rospy.Timer(rospy.Duration(0.2), self.timer_callback)

		

		return

	def timer_callback(self, data):
		self.base_to_stylus()
		self.kdl_forward_kinematics()

		return

	def get_joint_states(self,data):
		self.q_sensors = data.position

		return


	def base_to_stylus (self):

		try:
			(self.transl,self.quat) = self.tfListener.lookupTransform('base','stylus',rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
		self.rot= euler_from_quaternion(self.quat)
		self.transf = Twist(Vector3(self.transl[0],self.transl[1],self.transl[2]),(Vector3(self.rot[0],self.rot[1],self.rot[2])))
		self.Tsb.publish(self.transf)
		
		# self.test1 = "Publishing base to stylus"
		# print self.test1

		return


	def kdl_forward_kinematics (self):
		self.tree = kdl_tree_from_urdf_model(self.omni) # create a kdl tree from omni URDF model
		self.omni_kin = KDLKinematics(self.omni, "base", "stylus") # create instance that captures the kinematics of the robot arm 	

		#Forward Kinematics
		self.pose_stylus = self.omni_kin.forward(self.q_sensors) #compute the forward kinematics from the sensor joint angle position using the kinematics from the kdl tree

		#Inverse Kinematics
		self.q_guess = numpy.array(self.q_sensors)+0.2 #make an initial guess for your joint angles by deviating all the sensor joint angles by 0.2
		self.q_ik = self.omni_kin.inverse(self.pose_stylus, self.q_guess) #using the position from the forward kinematics 'pose_stylus' and the offset initial guess, compute 
		#the desired joint angles for that position.

		self.q_diff = self.q_ik-self.q_sensors

		#print "The joint states published from the Omni are", self.q_sensors
		print self.q_diff


		return

def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('omni_mini_proj')

    try:
        sim = OmniMiniProj()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()
