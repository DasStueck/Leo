#!/usr/bin/env python
#
# Revision $Id$



import rospy
from geometry_msgs.msg import Twist



	def __init__(self):

		velocity_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
		rospy.init_node('Moving', anonymous=True)
		rate = rospy.Rate(10)

		vel_msg = Twist()        
		vel_msg.angular.z = 0;
		vel_msg.linear.x = 0;



		
		rospy.sleep(0.5); 
		vel_msg.angular.z = 0;
					
		if vel_msg.linear.x < 0.1:
			vel_msg.linear.x = vel_msg.linear.x + 0.02;

		velocity_publisher.publish(vel_msg)




if __name__ == '__main__':
	try:
		Moving()

	except:
		rospy.loginfo("Move node terminated.")
