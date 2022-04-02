#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
import std_msgs.msg

import numpy as np

class MotionModel():
    #self.pubPWCov = rospy.Publisher("pwcov", PoseWithCovarianceStamped, queue_size=100)
    def __init__(self):
        # pose_topic_name = '/pwcov'
        self.poseWithCovraince = rospy.Subscriber('/pwcov',PoseWithCovarianceStamped,self.createModel)
        self.pub = rospy.Publisher("motion", Float64MultiArray ,queue_size=100)
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

    # def motionModel(self,omega = 10,phi=0):
    #     x_sensed = self.ballloc_xyz[0] 
    #     y_sensed = self.ballloc_xyz[1]
    #     z_sensed = self.ballloc_xyz[2]

    #     theta = np.arctan2(z_sensed, x_sensed)

    #     #Ackerman steering
    #     l = 0.5
    #     R_w = 0.1
    #     ratio = 8
    #     omega = 10 #need to figure out this omega can be populated
    #     v = R_w/ratio*omega

    #     dz = v*np.cos(theta)
    #     dx = v*np.sin(theta)
    #     dtheta = v/l * np.tan(phi)
    #     dy = np.random.normal(0, 0.2) #setting it 0.2 as a educated guess of random walk

    #     return [dx, dy, dz, dtheta]
       
    def motionModel(self,omegaL_k = 10, omegaR_k=10):
        x_sensed = self.ballloc_xyz[0] 
        y_sensed = self.ballloc_xyz[1]
        z_sensed = self.ballloc_xyz[2]

        theta = np.arctan2(z_sensed, x_sensed)

        #Differential steering
        l = 0.5
        R_leg = 0.1
        eta = 8        
        dt = 1
        
        front = R_leg/(2*eta)
        randomWalk = np.random.normal(y_sensed, 0.2) #setting it 0.2 as a educated guess of random walk

        Bmatrix = np.multiply(np.array([[front*np.cos(theta), front*np.cos(theta), 0], [front*np.sin(theta), front *np.sin(theta),0], [0, 0, 1], [-R_leg/(l*eta) ,R_leg/(l*eta), 0]]),dt)
        u_k = [omegaL_k,omegaR_k,randomWalk]
        Amatrix = np.array([[1, 0, 0, 0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        x_k = [x_sensed, y_sensed, z_sensed, theta]

        x_k1 = (x_k + np.matmul(Bmatrix,u_k) )
        return [x_k1, randomWalk]
    
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
        
        theta = np.arctan2(z_sensed, x_sensed)

        #observing it including noise
        x_observed = x_sensed + np.random.normal(0, np.sqrt(xVariance))
        y_observed = y_sensed + np.random.normal(0, np.sqrt(yVariance))
        z_observed = z_sensed + np.random.normal(0, np.sqrt(zVariance))

        #Dmatrix = np.array([x_observed],[y_observed],[z_observed])

        Cmatrix = np.array([[1, 0, 0, 0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        x_k = [x_sensed, y_sensed, z_sensed, theta]

        x_k1 = (x_k)

        return x_k1

    def createModel(self, data):
        self.ballloc_xyz = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z] 
        # print(self.ballloc_xyz)

if __name__ == "__main__":
    rospy.init_node("sensor_model")
    motionModel = MotionModel()
    viz_img = True
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # print("sendor model",motionModel.sensorModel())
        print(motionModel.motionModel())
        rate.sleep()