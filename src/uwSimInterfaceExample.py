#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

# TO-DO: Object Oriented version
# Remove globals
pub=[]
dat=[]
ok=False

#Navigation message callback.
def navCallback(data):
    global dat
    p=data.pose.pose.position
    if(not(p.x==0 and p.y==0 and p.z==0)):
		dat=data
		global ok
		ok=True

#Send speed to vehicle during t seconds.
def sendSpeed(x, y, z, yaw, t):
	global pub
	pub = rospy.Publisher('/dataNavigator', Odometry, queue_size=10)
	rate = rospy.Rate(25)
	i=0 
	while i<25*t:
		nav=Odometry()
		nav.twist.twist.linear.x=x
		nav.twist.twist.linear.y=y
		nav.twist.twist.linear.z=z
		nav.twist.twist.angular.z=yaw
		pub.publish(nav)
		rate.sleep()
		i+=1		

#Using Odometry, got to position sending speeds.
def sendPosition(x, y, z):
	global sub
	sub = rospy.Subscriber("/dataNavigator", Odometry, navCallback)
	while not ok:
		a=0	
	p=dat.pose.pose.position
	dx = x-p.x
	dy = y-p.y
	dz = z-p.z
	rate = rospy.Rate(15)
	while dx**2+dy**2+dz**2>0.2:
		sendSpeed(dx/3., dy/3., dz/3., 0, 0.2)
		rate.sleep()
		p=dat.pose.pose.position
		dx = x-p.x
		dy = y-p.y
		dz = z-p.z
	sub.unregister()

#Test functions
if __name__ == '__main__':
    try:
		rospy.init_node('navigator', anonymous=True)
		sendSpeed(0.6,0.6,0.1,0.3,3)
		sendPosition(3,3,3)
		rospy.spin()        
    except rospy.ROSInterruptException:
        pass
