#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

goldilocks = False
ball_grabbed= False
if __name__ == "__main__":
    rospy.init_node("goal_maker")
    goal = rospy.Publisher("move_base_simple/goal", PoseStamped ,queue_size=10)
    estop_pub = rospy.Publisher("/estop",Twist,queue_size=10)

    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)

    rate = rospy.Rate(10)
    frame ='d400_link'
    while not rospy.is_shutdown():
        try:
            (trans) = tfbuffer.lookup_transform('map', frame, rospy.Time()) #make 2d goal out of trans and rot
        except:
            continue
        if(goldilocks == False):
            # t =Twist()
            # t.linear.x = 1
            # t.linear.y = 0
            # t.linear.z = 0
            # t.angular.x = 0
            # t.angular.y = 0
            # t.angular.z = 0
            # estop_pub.publish(t)
            p = PoseStamped()
            p.header.frame_id ="map" 
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = 0.5
            p.pose.position.y = 0
            p.pose.position.z = 0
            p.pose.orientation.x = 0
            p.pose.orientation.y = 0
            p.pose.orientation.z = 0
            p.pose.orientation.w = 1
            goal.publish(p)
            print(math.sqrt((trans.transform.translation.x)**2 + (trans.transform.translation.y)**2))
            if (math.sqrt((trans.transform.translation.x)**2 + (trans.transform.translation.y)**2) <1.3):
                print('here1')
                goldilocks=True
        elif(goldilocks == True):
            t =Twist()
            t.linear.x = 0
            t.linear.y = 0
            t.linear.z = 0
            t.angular.x = 0
            t.angular.y = 0
            t.angular.z = 0
            estop_pub.publish(t)
            if (math.sqrt((trans.transform.translation.x)**2 + (trans.transform.translation.y)**2) >2):
                print('here')
                print(math.sqrt((trans.transform.translation.x)**2 + (trans.transform.translation.y)**2))
                t.linear.x = 1
                estop_pub.publish(t)
                ball_grabbed=True
        elif(ball_grabbed==True):
            p = PoseStamped()
            p.header.frame_id ="map" 
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = 5
            p.pose.position.y = -4
            p.pose.position.z = 0
            p.pose.orientation.x = 0
            p.pose.orientation.y = 0
            p.pose.orientation.z = 0
            p.pose.orientation.w = 1
            goal.publish(p)

    
        rate.sleep()