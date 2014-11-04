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
		
		#Create a tf listener
		self.tfListener = tf.TransformListener() 
		#tf.TransfromListener is a subclass that subscribes to the "/tf" message topic and adds transform 
		#data to the tf data structure. As a result the object is up to date with all current transforms

		#Create a publisher to a new topic name called "stylus_to_base_transf with a message type Twist"
		self.Tsb = rospy.Publisher('stylus_to_base_transf',Twist) 

		#Find the omni from the parameter server
		self.omni = URDF.from_parameter_server() 
		#Note: 	A node that initializes and runs the Phantom Omni has to be running in the background for 
		#		from_parameter_server to to find the parameter.
		#		Older versions of URDF label the function "load_from_parameter_server"


		#Subscribe to the "omni1_joint_states" topic, which provides the joint states measured by the omni in a
		# ROS message of the type sensor_msgs/JointState. Every time a message is published to that topic run the 
		#callback function self.get_joint_states, which is defined below.
		self.joint_state_sub = rospy.Subscriber("omni1_joint_states", JointState, self.get_joint_states)

		#OmniMiniProjTimers
		self.timer1 = rospy.Timer(rospy.Duration(0.2), self.timer_callback) 
		#Run a timer to manipulate the class structure as needed. The timer's 
		#callback function "timer_callback" then calls the desired functions

		return

	#The timer_callback runs every time timer1 triggers. In this example, it merely calls other functions. Generally,
	#it may include further calculations, just like any regular function.
	def timer_callback(self, data):

		self.base_to_stylus()
		self.kdl_forward_kinematics()

		return

	#get_joint_states extracts the joint agles from the JointState message  (angles, agular velocity, ...), which is
	#provided by the 'joint_state_sub' subscriber.
	def get_joint_states(self,data):

		self.q_sensors = data.position

		return

	#base_to_stylus uses tf to look up the transfrom from the frame located at the root of the Omni's URDF (/base) to the frame
	#at the end of the URDF (/stylus).
	def base_to_stylus (self):

		try:
			(self.transl,self.quat) = self.tfListener.lookupTransform('base','stylus',rospy.Time(0))
			#lookupTransrom is a method which returns the transfrom between two coordinate frames. 
			#lookupTransfrom returns a translation and a quaternion
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
			#If exceptions occur, pass skip trying to lookup the transform. 
			#It is encouraged to publish a logging message to rosout with rospy. 
			#i.e: 
			#rospy.logerr("Could not transform from %s to %s,"base","stylus")
			
		self.rot= euler_from_quaternion(self.quat) #Get euler angles from the quaternion
		self.transf = Twist(Vector3(self.transl[0],self.transl[1],self.transl[2]),(Vector3(self.rot[0],self.rot[1],self.rot[2])))
		#Store the transformation in a format compatible with gemoetr_msgs/Twist
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
        demo = OmniMiniProj()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()
