
#BACKUP 01.12.21 map Transform

#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
import actionlib
import move_base_msgs.msg
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from math import sqrt
from sensor_msgs.msg import LaserScan, CameraInfo, CompressedImage
import math
# from rpc_game_client.srv import PlayerScore -> Zum Fotos hochladen/schiessen
class LaserTag:
    def __init__(self):
        rospy.init_node('laser_tag') #Name des Nodes
        rospy.sleep(1)
        self.apriltag = None
        self.sortedDetections=[]
        # self.rpc_game_service = rospy.ServiceProxy("/rpc_score", PlayerScore)
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size=10) #Twists?/Move_Base
        self.sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.apriltag_cb)
        #self.sub_camera_info = rospy.Subscriber("camera/rgb/camera_info", CameraInfo, self.camera_info_cb)
       # self.compressed_image = rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, self.compressed_image_cb)
        #self.compressed_image = None
      #  self.camera_info = None
        
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)
        rospy.spin()

    def apriltag_cb(self,data):
        if not data.detections:
           rospy.loginfo('is empty')
        
        else:
            rospy.loginfo("YYYYYYYYYYYYYYYYYYYYYYYYeah")
            self.sortedDetections = sorted(data.detections, key=lambda tag: self.abstand(tag))
            for i in range (len(self.sortedDetections)):  
                rospy.loginfo("tag %d" % (i))      
                rospy.loginfo(self.sortedDetections[i].pose.pose.pose.position)
                rospy.loginfo("abstand: %f \n" % (self.abstand(self.sortedDetections[i])))
            self.apriltag = data
            rospy.loginfo(self.get_pose())

    def camera_info_cb(self,data):
        pass

    def compressed_image_cb(self,data):
        pass
            
        
    def get_pose(self):
        tag = PoseStamped()
        tag.header = self.apriltag.header
        tag.pose = self.sortedDetections[0].pose.pose.pose
        tag.header.stamp = rospy.Time(0)

        try:
            tag_bfp = self.tf_buffer.transform(tag, "map", timeout=rospy.Duration(1))
        except (tf2.ExtrapolationException) as e:
            rospy.logwarn(e)
            rospy.logerr('Severe transformation problem concerning the tag!')
            return None
        return tag_bfp

    def abstand(self, tag):
        return sqrt(tag.pose.pose.pose.position.z**2 + tag.pose.pose.pose.position.x**2) 

    

LaserTag()

