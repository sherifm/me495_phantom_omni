#!/usr/bin/env python  
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist,Vector3
from phantom_omni.msg import OmniFeedback

import numpy

class OmniMiniProj:

	def __init__ (self):

		self.tfListener = tf.TransformListener()

		self.timer1 = rospy.Timer(rospy.Duration(0.01), self.timer_callback) 

		self.force_pub = rospy.Publisher('/omni1_force_feedback',OmniFeedback)

		return

	def timer_callback (self, data):
		self.get_transforms()


		return

	def get_transforms (self):

		try:
			self.stylus = self.tfListener.lookupTransform('base','stylus',rospy.Time(0))
			self.floating = self.tfListener.lookupTransform('base','floating',rospy.Time(0))
			#lookupTransrom is a method which returns the transfrom between two coordinate frames. 
			#lookupTransfrom returns a translation and a quaternion
		except (tf.Exception):
			self.transf = None
			self.stylus = None
			pass
			#If exceptions occur, pass skip trying to lookup the transform. 
			#It is encouraged to publish a logging message to rosout with rospy. 
			#i.e: 
			#rospy.logerr("Could not transform from %s to %s,"base","stylus")

		if self.stylus != None:
			if self.floating != None:
				self.force_controller(self.stylus,self.floating)

		return

	def force_controller (self,pose_stylus,pose_floating):
		
		self.Kp = 10

		self.Fx_d = self.Kp*pose_floating[0][0]-self.Kp*pose_stylus[0][0]
		self.Fy_d = self.Kp*pose_floating[0][1]-self.Kp*pose_stylus[0][1]
		self.Fz_d = self.Kp*pose_floating[0][2]-self.Kp*pose_stylus[0][2]

		print self.Fx_d
		print self.Fy_d
		print self.Fz_d

		self.Fd = OmniFeedback(force=Vector3(self.Fy_d,self.Fz_d,self.Fx_d),position=Vector3(0,0,0))
		self.force_pub.publish(self.Fd)




def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('force_controller')

    try:
        force_demo = OmniMiniProj()
    except rospy.ROSInterruptException: pass

    rospy.spin()

if __name__=='__main__':
    main()