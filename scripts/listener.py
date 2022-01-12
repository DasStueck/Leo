#!/usr/bin/env python
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import math
import move
import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(data):
    haelfte1 = data.ranges[542:]
    haelfte2 = data.ranges[:541]
    minsHaelfte1 = [i for i in haelfte1 if  i >= 0.5]
    minsHaelfte2 = [i for i in haelfte2 if  i >= 0.5]
    minlinks = np.nanmin(minsHaelfte1)
    minrechts = np.nanmin(minsHaelfte2)
    #list(np.min(np.ma.masked_array(data.ranges, np.isnan(data.ranges))))
    rospy.loginfo("links: %s" % (minlinks))
    rospy.loginfo("rechts %s" % (minrechts))
    

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
	
    #if np.amin(mins) > 0.7:
        #Moving()


def listener():

    rospy.init_node('listener', anonymous=True)
		
    rospy.Subscriber('kinect_scan', LaserScan, callback)
	
    rospy.sleep(0.1);

#	while callback >1:
#		Moving(self)
#	else
#		rospy.loginfo("Wand.")
	

    rospy.spin()
	
		


if __name__ == '__main__':
    listener()
