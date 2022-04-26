#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
# import human_walk
import matplotlib.pyplot as plt
import pdb
import rospy
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf.transformations as tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import std_msgs.msg
from visualization_msgs.msg import Marker

class EKF():
    def __init__(self,nk,dt,X,U):
        self.nk = nk
        self.dt = 1
        self.X = X
        self.U = U
        
        self.prevX = [0,0]
        self.prevXSensed = [0, 0]

        self.Sigma_init = np.array([[0.05,0],[0,0.05]])     # <--------<< Initialize corection covariance
        self.sigma_measure = np.array([[0.05,0],[0,0.05]])  # <--------<< Should be updated with variance from the measurement
        self.KalGain = np.random.rand(2,2)                  # <--------<< Initialize Kalman Gain

        # self.Sigma_init = np.array([[0.05,0,0,0],[0,0.05,0,0],[0,0,0.05,0],[0,0,0,0.05]])     # <--------<< Initialize corection covariance
        # self.sigma_measure = np.array([[0.05,0,0,0],[0,0.05,0,0],[0,0,0.05,0],[0,0,0,0.05]])  # <--------<< Should be updated with variance from the measurement
        # self.KalGain = np.random.rand(4,4)                  # <--------<< Initialize Kalman Gain

        self.measurement_sub = rospy.Subscriber("/pwcov",PoseWithCovarianceStamped,self.measurement_cb) # <--------<< Subscribe to the ball pose topic
        self.correctedPos = rospy.Publisher("corrected_marker", Marker ,queue_size=100)
        self.future0 = rospy.Publisher("future0", Marker ,queue_size=100)
        self.future1 = rospy.Publisher("future1", Marker ,queue_size=100)
        self.future2 = rospy.Publisher("future2", Marker ,queue_size=100)
        self.future3 = rospy.Publisher("future3", Marker ,queue_size=100)
        self.correctedPWCOV = rospy.Publisher("correctedPWCOV", PoseWithCovarianceStamped, queue_size=100)

        self.ballloc_xyz = [0,0,0,0]
        self.covMatrix = [0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,]


        self.z_k = [0,0]
        self.futureVal = [[0,0]]
        self.considerReceivedValues = True
        #print(self.z_k)


        self.file1 = open("00000Demo.txt", "a")
        self.file2 = open("00000Covariance.txt", "a")
        self.file3 = open("00000Kalman.txt", "a")
        self.front = .2
        self.Sx_k_k = self.Sigma_init



    def prediction(self,x,U,Sigma_km1_km1):
        #TODO 
        # Use the motion model and input sequence to predict a n step look ahead trajectory. 
        # You will use only the first state of the sequence for the rest of the filtering process.
        # So at a time k, you should have a list, X = [xk, xk_1, xk_2, xk_3, ..., xk_n] and you will use only xk. 
        # The next time this function is called a lnew list is formed.
        xdot = self.dotX(x,U)
        X_list = []

        xk = np.add(x,xdot)
        Amatrix,Bmatrix = self.getGrad(xk,U)
        Atranspose = Amatrix.transpose()
        # motionNoise = np.array([[1.4,0,0,0],[0,1.4,0,0],[0,0,1.4,0],[0,0,0,1.4]]) #this is in x,y,z convert it to theta and omega
        montionNoiseRange = 1.4
        motionNoise = np.array([[np.random.normal(0, montionNoiseRange), 0],[0, np.random.normal(0, montionNoiseRange)]])
        Sigma_k_km1 = np.matmul(np.matmul(Amatrix,Sigma_km1_km1),Atranspose) + motionNoise
        self.predictNextSteps(xk,U,Sigma_k_km1,Amatrix,xdot)
        print(xk)
        return xk,Sigma_k_km1,Amatrix 
    
    
    def predictNextSteps(self,x_predicted_1,U,Sigma_km1_km1,A,xdot):
        x_next, S_next, A = [x_predicted_1, Sigma_km1_km1, A]
        montionNoiseRange = 1.4
        
        # B = np.array([[1,0],[0,1]])
        #print("-------------------------------------------------")
        self.futureVal = []
        for i in range(self.nk):
            # x_next, S_next, A_next = self.prediction(x_next, U, S_next)
            xdot_next = self.dotX(x_next, [x_next[0]*10, x_next[1]*10])
            x_next = np.add(x_next, xdot)
            #print(x_next)
            A, B = self.getGrad(x_next, [x_next[0]*10, x_next[1]*10])
            A_transpose = A.transpose()
            motionNoise = np.array([[np.random.normal(0, montionNoiseRange), 0],[0, np.random.normal(0, montionNoiseRange)]])
            S_next = np.matmul(np.matmul(A,S_next),A_transpose) + motionNoise
            self.futureVal.append([x_next[0],x_next[1]])


            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.Time.now()
            marker.id = 2
            marker.type = marker.SPHERE
            marker.action =marker.ADD
            marker.pose.position.x =  x_next[0]
            marker.pose.position.y =  self.ballloc_xyz[1]
            marker.pose.position.z = x_next[1]
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = .1
            marker.scale.y = .1
            marker.scale.z = .1
            marker.color.a = 1.0 
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            if i == 0:
                self.future0.publish(marker)
            if i == 1:
                self.future1.publish(marker)
            elif i == 2:
                self.future2.publish(marker)
            elif i == 3:
                self.future3.publish(marker)


    def correction(self,x_predict, Sx_k_km1, z_k, KalGain):
        #TODO
        # Write a function to correct your prediction using the observed state.
        I = np.array([[1,0],[0,1]])
        C = np.array([[1,0],[0,1]])

        #Correcting X
        temp = self.covMatrix
        xVariance = temp[0]
        yVariance = temp[7]
        zVariance = temp[14]

        h = [x_predict[0] + np.random.normal(0, np.sqrt(xVariance)),
            x_predict[1] + np.random.normal(0, np.sqrt(zVariance))]
        # z_k = [self.ballloc_xyz[0],self.ballloc_xyz[2]]
        temp = np.subtract(z_k , h)
        #print(temp)
        x_corrected = x_predict + np.matmul(KalGain, temp)

        #updating sigma
        Sx_k_k = np.matmul((I - np.matmul(KalGain, C)), Sx_k_km1)
        return x_corrected, Sx_k_k

    

    def update(self):
        X_input = self.X
        if self.considerReceivedValues == False:
            X_input = self.futureVal[0]
        X_predicted,Sx_k_km1, A = self.prediction(X_input,self.U,self.Sx_k_k)                        # PREDICTION STEP          
        self.z_k = [self.ballloc_xyz[0],self.ballloc_xyz[2]]
        X_corrected, self.Sx_k_k = self.correction(X_predicted, Sx_k_km1, self.z_k, self.KalGain)   # CORRECTION STEP 
        #print("----------------")
        #print(X_corrected)
        #print("----------------")
        #print(self.Sx_k_k)
        self.gainUpdate(Sx_k_km1)                                                                   # GAIN UPDATE       
        self.X = X_corrected 
        #print(self.X)
        # str1 = 
        self.file1.write("\nBall xyz: "+ str(str(self.ballloc_xyz[0]) + " , " + str(str(self.ballloc_xyz[2]))))
        self.file1.write("\npredictd: "+ str(self.X[0]) + " , "+ str(self.X[1]))
        self.file2.write("\n" + str(self.Sx_k_k))
        self.file3.write("\n" + str(self.KalGain))
        # self.file1.write(str(self.ballloc_xyz[0]) + ",\t"+ str(self.ballloc_xyz[2]) + ",\t" +str(self.X[0]) + ",\t" + str(self.X[1]) + "\n")
        # file1.close() 
        #print(self.ballloc_xyz)
        # self.X_correc = np.reshape(X_corrected,[6,2])   # <--------<< Publish 
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = 2
        marker.type = marker.SPHERE
        marker.action =marker.ADD
        marker.pose.position.x = self.X[0]
        marker.pose.position.y = self.ballloc_xyz[1]
        marker.pose.position.z = self.X[1]
        
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .1
        marker.color.a = 1.0 
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.correctedPos.publish(marker)

        tempMatrix = [(self.Sx_k_k[0][0]),0,(self.Sx_k_k[0][1]),0,0,0,
                    0,0,0,0,0,0,
                    (self.Sx_k_k[1][0]),0,(self.Sx_k_k[1][1]),0,0,0,
                    0,0,0,0,0,0,
                    0,0,0,0,0,0,
                    0,0,0,0,0,0]

        pwc = PoseWithCovarianceStamped()
        pwc.header.frame_id = "camera_link"
        pwc.header.stamp = rospy.Time.now()
        pwc.pose.pose.position.x = self.X[0]
        pwc.pose.pose.position.y = self.ballloc_xyz[1]
        pwc.pose.pose.position.z = self.X[1]

        thetaOrient = np.arctan2(self.X[1], self.X[0])
        Xorient, Yorient, Zorient, Worient = tf.quaternion_from_euler(0,0, thetaOrient)
                
        pwc.pose.pose.orientation.x = Xorient
        pwc.pose.pose.orientation.y = Yorient
        pwc.pose.pose.orientation.z = Zorient
        pwc.pose.pose.orientation.w = Worient
        pwc.pose.covariance = tempMatrix
        self.correctedPWCOV.publish(pwc)


        #self.X = self.X_correc
        self.prevX = X_corrected #self.X 

        

    def gainUpdate(self,Sx_k_km1):
        #TODO
        # Write a function to update the Kalman Gain, a.k.a. self.KalGain
        temp = self.covMatrix

        #for 2D 
        #TODO change the noise to correspond to the futureVals
        xNoise = 0
        zNoise = 0
        if self.considerReceivedValues is True:
            temp = self.covMatrix    
            xNoise = np.random.normal(0, np.sqrt(temp[0]))
            zNoise = np.random.normal(0, np.sqrt(temp[14]))
        else:
            xVal, zVal = self.futureVal[0]
            xNoise = np.random.normal(0, self.get_x_variance(xVal))
            zNoise = np.random.normal(0, self.get_z_variance(zVal))
        
        sensorNoise = np.array([[xNoise,0],[0,zNoise]])
        C = np.array([[1,0],[0,1]])
        

        tempBracket = np.matmul(np.matmul(C,Sx_k_km1),C.transpose()) + sensorNoise
        invTempBracket = inv(tempBracket)
        self.KalGain = np.matmul(Sx_k_km1,(np.matmul(C.transpose(), invTempBracket)))
        return self.KalGain



    
    def dotX(self,x,U):
        x_sensed = x[0]
        z_sensed = x[1]
        X_sensed = [x_sensed, z_sensed]

        omegaL_k, omegaR_k = U
        # xDotk = (np.subtract(X_sensed, self.prevX)) / self.dt
        xDotk = (np.subtract(X_sensed, self.prevX)) / self.dt
        
        self.prevXSensed = X_sensed
        """
        l = 0.5
        R_leg = 0.1
        eta = 8        
        dt = self.dt
        omegaL_k, omegaR_k = U
        front = R_leg/(2*eta)
        randomWalk = np.random.normal(y_sensed, 0.2) #setting it 0.2 as a educated guess of random walk

        Bmatrix = np.multiply(np.array([[front*np.cos(theta), front*np.cos(theta), 0], [front*np.sin(theta), front *np.sin(theta),0], [0, 0, 1], [-R_leg/(l*eta) ,R_leg/(l*eta), 0]]),dt)
        u_k = [omegaL_k,omegaR_k,randomWalk]
        Amatrix = np.array([[1, 0, 0, 0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        x_k = [x_sensed, y_sensed, z_sensed, theta]

        x_k1 = (x_k + np.matmul(Bmatrix,u_k) )
        xDotk = x_k1 - x_k""" 
        return xDotk

    def getGrad(self,x,U):
        # TODO
        # Linearize the motion model here. It should be called in the self.predict function and should yield the A and B matrix.
        x_sensed = x[0] 
        # y_sensed = x[1]
        z_sensed = x[1]

        theta = np.arctan2(z_sensed, x_sensed)
        
        #Modelling the differential steering
        l = 1
        R_leg = 1
        eta = 1
        dt = self.dt
        front = R_leg/(2*eta)

        # Amatrix = np.multiply(np.array([[front*np.cos(theta), front*np.cos(theta), 0, 0],
        #                                 [front*np.sin(theta), front *np.sin(theta),0, 0],
        #                                 [0, 0, 1, 0], 
        #                                 [-R_leg/(l*eta) ,R_leg/(l*eta), 0, 0]]),dt)
        # Bmatrix = np.array([[1, 0, 0, 0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        
        #for 2D
        front = self.front
        # print(front)
        #Previous MotionModel
        # Amatrix = np.multiply(np.array([[front*np.cos(theta), front*np.cos(theta)],
        #                                 [front*np.sin(theta), front *np.sin(theta)]]),dt)
        xChange = x_sensed - self.prevX[0]
        zChange = z_sensed - self.prevX[1]
        Amatrix = np.multiply(np.array([[xChange,0],
                                        [0,zChange]]),dt)
        Bmatrix = np.array([[1,0],[0,1]])           

        return [Amatrix,Bmatrix]

    def get_x_variance(self, x):
        temp = 6.07E-06*x + 1.76E-04
        return abs(temp)

    def get_z_variance(self, z):
        temp = ((0.013*z)-0.0021)
        return abs(temp)

    def measurement_cb(self, data):
        theta = np.arctan2(data.pose.pose.position.z, data.pose.pose.position.x)
        self.ballloc_xyz = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,theta]
        self.U = [int(data.pose.pose.position.x*10), int(data.pose.pose.position.z*10)]
        self.covMatrix = data.pose.covariance
        self.considerReceivedValues = True
        if abs(self.ballloc_xyz[0])>5 or abs(self.ballloc_xyz[2])>5:
            self.considerReceivedValues = False
            print("not considering values")
        self.X = [self.ballloc_xyz[0],self.ballloc_xyz[2]]

if __name__ == '__main__':
    rospy.init_node("EKF")
    rate = rospy.Rate(4)

    # ---------------Define initial conditions --------------- #
    nk = 4       # <------<< Look ahead duration in seconds
    dt = 1       # <------<< Sampling duration of discrete model
    X =  [0,0]       # <------<< Initial State of the Ball
    U =  [15,15]       # <------<< Initial input to the motion model; U = [omegaL_k, omegaR_k]
    
    filter = EKF(nk,dt,X,U)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
    
        filter.update()     
        rate.sleep()
 