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
from tf.transformations import compose_matrix 
from tf.transformations import is_same_transform
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
		#Run a timer to manipulate the class structure as needed. The timer's 
		#callback function "timer_callback" then calls the desired functions
		self.timer1 = rospy.Timer(rospy.Duration(0.01), self.timer_callback) 
		
		#Create a separate slower timer on a lower frequency for a comfortable print out
		self.print_timer = rospy.Timer(rospy.Duration(1), self.print_output) 

		return

	#The timer_callback runs every time timer1 triggers. In this example, it merely calls other functions. Generally,
	#it may include further calculations, just like any regular function.
	def timer_callback(self, data):
		self.tf_base_to_stylus()
		return

	#get_joint_states extracts the joint agles from the JointState message  (angles, agular velocity, ...), which is
	#provided by the 'joint_state_sub' subscriber.
	def get_joint_states(self,data):
		try:
			q_sensors = data.position
		except rospy.ROSInterruptException: 
			self.q_sensors = None
		pass

		if q_sensors != None:
			self.kdl_kinematics(q_sensors)

		return

	#tf_base_to_stylus uses tf to look up the transfrom from the frame located at the root of the Omni's URDF (/base) to the frame
	#at the end of the URDF (/stylus).
	def tf_base_to_stylus (self):

		try:
			(self.transl,self.quat) = self.tfListener.lookupTransform('base','stylus',rospy.Time(0))
			#lookupTransrom is a method which returns the transfrom between two coordinate frames. 
			#lookupTransfrom returns a translation and a quaternion
			self.rot= euler_from_quaternion(self.quat) #Get euler angles from the quaternion

			self.tf_SE3 = compose_matrix(angles=self.rot,translate=self.transl)

			#Store the transformation in a format compatible with gemoetr_msgs/Twist
			self.transf = Twist(Vector3(self.transl[0],self.transl[1],self.transl[2]),(Vector3(self.rot[0],self.rot[1],self.rot[2])))
			#Publish the transformation
			self.Tsb.publish(self.transf)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			self.tf_SE3 = None
			pass
			#If exceptions occur, skip trying to lookup the transform. 
			#It is encouraged to publish a logging message to rosout with rospy. 
			#i.e: 
			#rospy.logerr("Could not transform from %s to %s,"base","stylus")

		return


	def kdl_kinematics (self,data):

		self.q_sensors = data
		self.tree = kdl_tree_from_urdf_model(self.omni) # create a kdl tree from omni URDF model
		self.omni_kin = KDLKinematics(self.omni, "base", "stylus") # create instance that captures the kinematics of the robot arm 	

		#Forward Kinematics
		self.pose_stylus = self.omni_kin.forward(data) #compute the forward kinematics from the sensor joint angle position using the kinematics from the kdl tree


		#Inverse Kinematics
		self.q_guess = numpy.array(data)+0.2 #make an initial guess for your joint angles by deviating all the sensor joint angles by 0.2
		self.q_ik = self.omni_kin.inverse(self.pose_stylus, self.q_guess) #using the position from the forward kinematics 'pose_stylus' and the offset initial guess, compute 
		#the desired joint angles for that position.

		self.delta_q = self.q_ik-data


	def print_output (self,data):

		if self.tf_SE3 != None and self.q_sensors !=None:
			print "Base to stylus using tf:","\n", self.tf_SE3, "\n"
			print "base to stylus using KDL forward kinematics", "\n" , self.pose_stylus, "\n"
			print "transformations.is.same_transform tf vs KDL forward kin:", "\n", is_same_transform(self.tf_SE3,self.pose_stylus), "\n"
			print "Joint angles from sensors q_sensors:","\n" , self.q_sensors, "\n"
			print "Joint state angles from KDL inverse kinematics q_ik", "\n" , self.q_ik , "\n"
			print "The difference delta_q between q_sensors and q_ik", "\n" , self.delta_q , "\n\n\n\n\n\n\n\n\n"



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
