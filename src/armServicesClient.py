#!/usr/bin/env python
from arm5e_arm_services.srv import GetJoints
from arm5e_arm_services.srv import ForwardKinematics
from arm5e_arm_services.srv import InverseKinematics
from arm5e_arm_services.srv import MoveArm
from pcl_manipulation.srv import DetectObject
import rospy

# TO-DO: Object Oriented version

#Get current joint state (position)
def callGetJoints():
	rospy.wait_for_service('get_joints')
	get_joints = rospy.ServiceProxy('get_joints', GetJoints)
	try:
		res = get_joints()
		#print("Joint positions" + str(res.joints.position))
		return res.joints.position
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		
#Call Direct Kinemaics joints=>result_pose	
def callFK(j):
	rospy.wait_for_service('forward_kinematics')
	forward_kinematics = rospy.ServiceProxy('forward_kinematics', ForwardKinematics)
	try:
		res = forward_kinematics(j)
		print("bMe is: " + str(res))
		return res.bMe
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		
#Call Inverse Kinemaics pose=>joints,result_pose	
def callIK( req ):
	rospy.wait_for_service('inverse_kinematics')
	inverse_kinematics = rospy.ServiceProxy('inverse_kinematics', InverseKinematics)
	try:
		res = inverse_kinematics(req)
		print("Joint positions for desired bMe: " + str(res))
		return res
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
		
#Move arm at a certain joint speed		
def callMoveArm( vel ):
	rospy.wait_for_service('move_arm')
	move_arm = rospy.ServiceProxy('move_arm', MoveArm)
	try:
		res = move_arm(vel)
		#print("Moving arm...")
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))		

#Move arm to a certain joint configuration
def callMoveArmTo( position ):
	cj=callGetJoints()
	spd=[position[0]-cj[0],position[1]-cj[1],position[2]-cj[2],position[3]-cj[3],position[4]-cj[4]]
	d=(position[0]-cj[0])**2+(position[1]-cj[1])**2+(position[2]-cj[2])**2+(position[3]-cj[3])**2+(position[4]-cj[4])**2
	while d>=0.05:
		cj=callGetJoints()
		spd=[position[0]-cj[0],position[1]-cj[1],position[2]-cj[2],position[3]-cj[3],position[4]-cj[4]]
		d=(position[0]-cj[0])**2+(position[1]-cj[1])**2+(position[2]-cj[2])**2+(position[3]-cj[3])**2+(position[4]-cj[4])**2
		callMoveArm(spd)
	
#Use object detection to retrieve object position
def callDetectObject():
	rospy.wait_for_service('detect_object')
	detect_object = rospy.ServiceProxy('detect_object', DetectObject)
	try:
		# Known parameters, must allow change and tunning.
		res = detect_object(0.3,1.57,0) 
		print("Object position:" + str(res))
		return res
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))				
	
print("Lets test some services...: ")

#Move ARM (should be done for a while)
callMoveArm([-0.1,-0.1,0,0,0])

#Test Kinematics
current_joints=callGetJoints()
eef_pose=callFK(current_joints)
callIK(eef_pose)

#Object detection
cylinder=callDetectObject()
arm_config=callIK(cylinder.cMo)
bMe=callFK(arm_config.out_joints)
callMoveArmTo(arm_config.out_joints)
