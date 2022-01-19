#!/usr/bin/env python

import rospy
import time
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
import actionlib
import move_base_msgs.msg 
from move_base_msgs.msg import MoveBaseGoal 
import tf2_ros as tf2
from tf2_geometry_msgs import PoseStamped
from math import sqrt
from sensor_msgs.msg import LaserScan, CameraInfo, CompressedImage
import math
from rpc_game_client.srv import PlayerScore # FOTOS SCHIESSEN UND SO
class LaserTag:
    def __init__(self):
        rospy.init_node('laser_tag') #Name des Nodes
        rospy.sleep(1)

        self.startmove=True
        self.sortedDetections=[]
        self.posNo = 0
        self.letzterSchuss = 0
        
        #PlayerScore Sachen
        self.rpc_game_service = rospy.ServiceProxy("/rpc_score", PlayerScore)
        self.apriltag = None
        self.sub = rospy.Subscriber("tag_detections", AprilTagDetectionArray, self.apriltag_cb)
        self.sub_camera_info = rospy.Subscriber("camera/rgb/camera_info", CameraInfo, self.camera_info_cb)
        self.compressed_image = rospy.Subscriber("/camera/rgb/image_color/compressed", CompressedImage, self.compressed_image_cb)
        self.compressed_image = None
        self.camera_info = None

	#CostMap-Clear
	#self.move_base_service = rospy.ServiceProxy("/move_base")
	#self.move_base_service.

        #Transformkram
        self.tf_buffer = tf2.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2.TransformListener(self.tf_buffer)



        #Moving Client Stuff
        self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        self.client.wait_for_server()



        #Initialposen
        self.sortedDetections = None
        self.base_poses = []   
        rospy.sleep(1)
        rospy.loginfo("Intialisieren")     
        self.base_points = [(4.5,-7),(4.5,-7)]
        rospy.sleep(1)
        self.start_movement()

        self.pose_movement()

        rospy.spin()

    def pose_movement(self):
        self.posNo = (self.posNo+1)%len(self.base_poses)  
        self.moving(self.base_poses[self.posNo])
        rospy.loginfo("pose_movement")
        



    def apriltag_cb(self,data):
    # rospy.loginfo(data.detections)
        if data.detections:
            aktuelleZeit = rospy.get_time()
            
            if aktuelleZeit - self.letzterSchuss >= 3:            
                rospy.loginfo("april tag gefunden") 
                self.sortedDetections = sorted(data.detections, key=lambda tag: self.abstand(tag))
                rospy.loginfo(self.abstand(self.sortedDetections[0]))
                rospy.loginfo(data.detections)
                self.apriltag = data
                if self.abstand(self.sortedDetections[0]) < 1:
                    rospy.loginfo("Foto geschossen")
                    self.knipsen() 
                    self.letzterSchuss = rospy.get_time() 
                    return               
                rospy.loginfo(self.sortedDetections[0].id)
                self.moving(self.get_pose())
                self.apriltag = None  
                self.sortedDetections = None             
                rospy.loginfo("angekommen")
                #data.detections = None
                self.pose_movement()
            else:
                 rospy.loginfo("Zeit bis naechster Schuss: %f" % ( 10 - (aktuelleZeit - self.letzterSchuss))) 
        else:
            rospy.loginfo("is empty")
   
    def camera_info_cb(self,data):
        self.camera_info = data

    def compressed_image_cb(self,data):
        self.compressed_image = data
       
    def knipsen(self):
        ps = self.rpc_game_service(self.compressed_image,self.camera_info)
        rospy.loginfo(ps)
        rospy.loginfo('ps')
        self.sortedDetections = None
        self.apriltag = None
        



    def start_movement(self):
        for point in self.base_points:
            #self.base_poses #Neu und ungetestet
            self.base_poses.append(PoseStamped())
            self.base_poses[-1].pose.position.x = point[0];
            self.base_poses[-1].pose.position.y = point[1];
            self.base_poses[-1].header.frame_id = 'map'
            self.base_poses[-1].pose.orientation.w = 1;
            rospy.loginfo(self.base_poses[-1])
            #Wir Geben: PoseStamped[]


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

    
    def moving(self, gp):

        rospy.loginfo("Moving")
        gp.header.stamp = rospy.Time(0)
        tag_pose = MoveBaseGoal()
        tag_pose.target_pose.pose.position.x = gp.pose.position.x;
        tag_pose.target_pose.pose.position.y = gp.pose.position.y;
        tag_pose.target_pose.pose.orientation.w = gp.pose.orientation.w;
        tag_pose.target_pose.header.frame_id = 'map'
        self.client.send_goal(tag_pose)
        #rospy.sleep(1)
        rospy.loginfo("waiting for result")
        self.client.wait_for_result()
        rospy.loginfo("moving beendet")
        
        #rospy.sleep(1)

           
        # return client.get_result()

        
    

LaserTag()

