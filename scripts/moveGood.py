#!/usr/bin/env python
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import math
import move
import rospy
import numpy as np
from geometry_msgs.msg import Twist

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan





def callback(data):
    velocity_publisher = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
    vel_msg = Twist()  

    haelfte1 = data.ranges[542:]
    haelfte2 = data.ranges[:541]
    #minsHaelfte1 = [i for i in haelfte1 if  i >= 0.5]
    #minsHaelfte2 = [i for i in haelfte2 if  i >= 0.5]
    minsHaelfte1 = [i for i in haelfte1 if not math.isnan(i)]
    minsHaelfte2 = [i for i in haelfte2 if not math.isnan(i)]
    minlinks = min(minsHaelfte1)
    minrechts = min(minsHaelfte2)
    #list(np.min(np.ma.masked_array(data.ranges, np.isnan(data.ranges))))
    rospy.loginfo("links: %s" % (minlinks))
    rospy.loginfo("rechts %s" % (minrechts))
    

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    

    vel_msg.angular.z = 0;
    vel_msg.linear.x = 0;

    if (minlinks < 1 or minrechts < 1):

        if minlinks < 1:
            rospy.loginfo("Fuer links entschieden %s" % (minlinks))
            vel_msg.angular.z = 0.4
            velocity_publisher.publish(vel_msg)           	
        
        else:
            vel_msg.angular.z = -0.4;
            rospy.loginfo("Fuer rechts entschieden %s" % (minrechts))
            velocity_publisher.publish(vel_msg)

    elif (minlinks and minrechts) > 1:
        vel_msg.angular.z = 0;
        vel_msg.linear.x = 0.2;
        velocity_publisher.publish(vel_msg)	
     
	


def listener():

    rospy.init_node('listener', anonymous=True)
		
    rospy.Subscriber('kinect_scan', LaserScan, callback)

#	while callback >1:
#		Moving(self)
#	else
#		rospy.loginfo("Wand.")
	

    rospy.spin()
	
		


if __name__ == '__main__':
    listener()
