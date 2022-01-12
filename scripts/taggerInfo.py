#!/usr/bin/env python
import rospy
#import geometry_msgs.msg
# msg type
from math import sqrt
from apriltag_ros.msg import AprilTagDetectionArray

def abstand(tag):
    return sqrt(tag.pose.pose.pose.position.z**2 + tag.pose.pose.pose.position.x**2) 
def callback(data):
    if not data.detections:
        rospy.loginfo('is empty')
    else:


        sortedDetections = sorted(data.detections, key=lambda tag: abstand(tag))
        for i in range (len(sortedDetections)):  
            rospy.loginfo(sortedDetections[i].header)
            #rospy.loginfo("tag %d" % (i))      
            #rospy.loginfo(sortedDetections[i].pose.pose.pose.position)
            rospy.loginfo("abstand: %f \n" % (abstand(sortedDetections[i])))
def tagListener():
    rospy.Subscriber('tag_detections', AprilTagDetectionArray, callback)
   # tag_publisher = rospy.Publisher('tagPositionierer') #,msg type#)
    rospy.spin()
if __name__ == '__main__':
    rospy.init_node('tagInfo', anonymous=True)
    rospy.sleep(0.1)
    rospy.loginfo(rospy.get_time())
    tagListener()

