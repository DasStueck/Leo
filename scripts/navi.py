#!/usr/bin/env python

import rospy
import actionlib
import move_base_msgs.msg
from move_base_msgs.msg import MoveBaseGoal 


def moving():


    client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
 
    rospy.loginfo('client defined')
   
    client.wait_for_server()

    rospy.loginfo('client wait for server')



    goal_pose00 = MoveBaseGoal()

    goal_pose00.target_pose.pose.position.x = 3.0;
    goal_pose00.target_pose.pose.position.y = -2.0;
    goal_pose00.target_pose.pose.orientation.w = 1.0;
    goal_pose00.target_pose.header.frame_id = 'map'

    rospy.loginfo('goal00 defined')    

    goal_pose01 = MoveBaseGoal()
    goal_pose01.target_pose.pose.position.x = 5.0;
    goal_pose01.target_pose.pose.position.y = -2.0;
    goal_pose01.target_pose.pose.orientation.w = 1.0;
    goal_pose01.target_pose.header.frame_id = 'map'
    rospy.loginfo('goal01 defined')    

    for i in range(5):
        
        client.send_goal(goal_pose00)
        rospy.loginfo('send goalpose00')

        client.wait_for_result()
        rospy.loginfo('waiting for result') 

        client.send_goal(goal_pose01)

        rospy.loginfo('send goalpose01')

        client.wait_for_result()
        rospy.loginfo('waiting for result') 

    
    
    rospy.loginfo('schleife durch') 
    
    return client.get_result()
 

    
    

if __name__ == '__main__':
    #try:
    rospy.init_node('navi_py')
    rospy.sleep(1);
    moving();

    #except *SE:
       # rospy.loginfo("Ein Error.")



