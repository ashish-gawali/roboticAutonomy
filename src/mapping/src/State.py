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
from visualization_msgs.msg import Marker

goldilocks = False
ball_grabbed= False
class NavStack():


    def __init__(self):          
        self.goal = rospy.Publisher("move_base_simple/goal", PoseStamped ,queue_size=10)
        self.marker_sub = rospy.Subscriber('/visualization_marker',Marker,self.marker_cb) 
    
    def goal_pub(self,val):
            p = PoseStamped()
            p.header.frame_id ="map" 
            p.header.stamp = rospy.Time.now()
            p.pose.position.x = val[0]
            p.pose.position.y = val[1]
            p.pose.position.z = val[2]
            p.pose.orientation.x = 0
            p.pose.orientation.y = 0
            p.pose.orientation.z = 0
            p.pose.orientation.w = 1
            self.goal.publish(p)
            print('here')

    def marker_cb(self,data):
        markerPoints = [data.pose.position.x,data.pose.position.y,data.pose.position.z]
        print(markerPoints)
        self.goal_pub(markerPoints)

if __name__ == "__main__":
    rospy.init_node("goal_maker")


    nav= NavStack()
    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)

    rate = rospy.Rate(10)
    frame ='d400_link'
    while not rospy.is_shutdown():    
        rate.sleep()