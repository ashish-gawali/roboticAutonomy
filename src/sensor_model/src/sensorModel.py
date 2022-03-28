#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np

class MotionModel():
    #self.pubPWCov = rospy.Publisher("pwcov", PoseWithCovarianceStamped, queue_size=100)
    def __init__(self):
        # pose_topic_name = '/pwcov'
        self.poseWithCovraince = rospy.Subscriber('/pwcov',PoseWithCovarianceStamped,self.createModel)
        self.ballloc_xyz = [0, 0, 0]
    
    def get_x_variance(self):
        temp = 6.07E-06*self.ballloc_xyz[0] + 1.76E-04
        return abs(temp)

    def get_y_variance(self):
        temp = 2.97E-04*self.ballloc_xyz[1] + -2.49E-03
        return abs(temp)
    
    def get_z_variance(self):
        temp = ((0.013*self.ballloc_xyz[2])-0.0021)
        return abs(temp)

    #motion model
    # for omega need to figure out this omega can be populated

    def motionModel(self,omega = 10,phi=0):
        x_sensed = self.ballloc_xyz[0] 
        y_sensed = self.ballloc_xyz[1]
        z_sensed = self.ballloc_xyz[2]

        theta = np.arctan2(z_sensed, x_sensed)

        #Ackerman steering
        l = 0.5
        R_w = 0.1
        ratio = 8
        omega = 10 #need to figure out this omega can be populated
        v = R_w/ratio*omega

        dz = v*np.cos(theta)
        dx = v*np.sin(theta)
        dtheta = v/l * np.tan(phi)
        dy = np.random.normal(0, 0.2) #setting it 0.2 as a educated guess of random walk

        return [dx, dy, dz, dtheta]
    
    #gives output as observed values including noise [x,y,z]
    def sensorModel(self):
        #getting variance based on where we sensed the ball
        xVariance = self.get_x_variance()
        yVariance = self.get_y_variance()
        zVariance = self.get_z_variance()

        #sensed using camera
        x_sensed = self.ballloc_xyz[0] 
        y_sensed = self.ballloc_xyz[1]
        z_sensed = self.ballloc_xyz[2]
        
        #observing it including noise
        x_observed = x_sensed + np.random.normal(0, np.sqrt(xVariance))
        y_observed = y_sensed + np.random.normal(0, np.sqrt(yVariance))
        z_observed = z_sensed + np.random.normal(0, np.sqrt(zVariance))

        return [x_observed, y_observed, z_observed]

    def createModel(self, data):
        self.ballloc_xyz = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z] 
        # print(self.ballloc_xyz)

if __name__ == "__main__":
    rospy.init_node("sensor_model")
    motionModel = MotionModel()
    viz_img = True
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("sendor model",motionModel.sensorModel())
        print("motion model",motionModel.motionModel())
        rate.sleep()