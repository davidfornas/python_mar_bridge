#!/usr/bin/env python

from arm5e_arm_services.srv import GetJoints
from arm5e_arm_services.srv import ForwardKinematics
from arm5e_arm_services.srv import InverseKinematics
from arm5e_arm_services.srv import MoveArm
from pcl_manipulation.srv import DetectObject
#from beginner_tutorials.srv import *
import rospy

def callGetJoints():
	rospy.wait_for_service('get_joints')
	get_joints = rospy.ServiceProxy('get_joints', GetJoints)
	try:
		res = get_joints()
		print("Joint positions" + str(res.joints.position))
		return res.joints.position
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		
def callFK(j):
	rospy.wait_for_service('forward_kinematics')
	forward_kinematics = rospy.ServiceProxy('forward_kinematics', ForwardKinematics)
	try:
		res = forward_kinematics(j)
		print("bMe is: " + str(res))
		return res.bMe
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		
		
def callIK( req ):
	rospy.wait_for_service('inverse_kinematics')
	inverse_kinematics = rospy.ServiceProxy('inverse_kinematics', InverseKinematics)
	try:
		res = inverse_kinematics(req)
		print("Joint positions for desired bMe: " + str(res))
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
				
def callMoveArm( vel ):
	rospy.wait_for_service('move_arm')
	move_arm = rospy.ServiceProxy('move_arm', MoveArm)
	try:
		res = move_arm(vel)
		#print("Moving arm...")
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))		

	
def callDetectObject():
	rospy.wait_for_service('detect_object')
	detect_object = rospy.ServiceProxy('detect_object', DetectObject)
	try:
		res = detect_object()
		print("Object position:")
		return res
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))		
			
		
	
print("Lets test some services...: ")
#while 1<2:
#	callMoveArm([-0.1,-0.1,0,0,0])

j=callGetJoints()
b=callFK(j)
callIK(b)
callDetectObject()
#while 1<2:
#	
#	
#	callMoveArm([0.1,0.1,0,0,0])
