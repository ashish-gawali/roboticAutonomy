#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
# import human_walk
import matplotlib.pyplot as plt
import pdb
import rospy
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import std_msgs.msg

class EKF():
    def __init__(self,nk,dt,X,U):
        self.nk = nk
        self.dt = 1
        self.X = X
        self.U = U
        
        self.prevX = [0,0,0,0]

        # self.Sigma_init = np.array([[0.05,0],[0,0.05]])     # <--------<< Initialize corection covariance
        # self.sigma_measure = np.array([[0.05,0],[0,0.05]])  # <--------<< Should be updated with variance from the measurement
        # self.KalGain = np.random.rand(2,2)                  # <--------<< Initialize Kalman Gain

        self.Sigma_init = np.array([[0.05,0,0,0],[0,0.05,0,0],[0,0,0.05,0],[0,0,0,0.05]])     # <--------<< Initialize corection covariance
        self.sigma_measure = np.array([[0.05,0,0,0],[0,0.05,0,0],[0,0,0.05,0],[0,0,0,0.05]])  # <--------<< Should be updated with variance from the measurement
        self.KalGain = np.random.rand(4,4)                  # <--------<< Initialize Kalman Gain

        self.measurement_sub = rospy.Subscriber("/pwcov",PoseWithCovarianceStamped,self.measurement_cb) # <--------<< Subscribe to the ball pose topic
        self.ballloc_xyz = [0,0,0,0]
        self.covMatrix = [0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,]


        self.z_k = [0,0,0,0]
        #print(self.z_k)

        self.Sx_k_k = self.Sigma_init



    def prediction(self,x,U,Sigma_km1_km1):
        #TODO 
        # Use the motion model and input sequence to predict a n step look ahead trajectory. 
        # You will use only the first state of the sequence for the rest of the filtering process.
        # So at a time k, you should have a list, X = [xk, xk_1, xk_2, xk_3, ..., xk_n] and you will use only xk. 
        # The next time this function is called a lnew list is formed.
        xdot = self.dotX(x,U)
        # print(x)
        for i in range( self.nk):
            # X[i] = np.add(x, (xdot*self.dt*i))
            # print(i)
            # print("XDot")
            # print(xdot)
            # print("Xdot mul")
            # print((xdot.dot(self.dt*i)))
            # print("x")
            # print(x)
            X[i] = np.add(x, (xdot.dot(self.dt*i)))
            #print(X[i])

        xk = X[0]
        # print("xk")
        # print(xk)
        Amatrix,Bmatrix = self.getGrad(xk,U)
        Atranspose = Amatrix.transpose()
        motionNoise = np.array([[1.4,0,0,0],[0,1.4,0,0],[0,0,1.4,0],[0,0,0,1.4]]) #this is in x,y,z convert it to theta and omega
        Sigma_k_km1 = np.matmul(np.matmul(Amatrix,Sigma_km1_km1),Atranspose) + motionNoise

        return xk,Sigma_k_km1,Amatrix 


    def correction(self,x_predict, Sx_k_km1, z_k, KalGain):
        #TODO
        # Write a function to correct your prediction using the observed state.
        I = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        C = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
        
        #Correcting X
        temp = self.covMatrix
        xVariance = temp[0]
        yVariance = temp[7]
        zVariance = temp[14]

        h = [self.ballloc_xyz[0] + np.random.normal(0, np.sqrt(xVariance)),
            self.ballloc_xyz[1] + np.random.normal(0, np.sqrt(yVariance)),
            self.ballloc_xyz[2] + np.random.normal(0, np.sqrt(zVariance)),
            0]   #as we do not have theta noise
        
        temp = np.subtract(z_k , h)
        x_corrected = x_predict + np.matmul(KalGain, temp)


        #updating sigma
        Sx_k_k = np.matmul((I - np.matmul(KalGain, C)), Sx_k_km1)

        return x_corrected, Sx_k_k


    def update(self):
        self.X_pred = self.X 
        print(self.X)
        X_predicted,Sx_k_km1, A = self.prediction(self.X,self.U,self.Sx_k_k)                        # PREDICTION STEP  
        X_corrected, self.Sx_k_k = self.correction(X_predicted, Sx_k_km1, self.z_k, self.KalGain)   # CORRECTION STEP 
        print("----------------")
        print(X_corrected)
        print("----------------")
        self.gainUpdate(Sx_k_km1)                                                                   # GAIN UPDATE       
        self.X = X_corrected 
        file1 = open("MyFile.txt", "a")
        # str1 = 
        file1.write("\nBall xyz: "+ str(self.ballloc_xyz))
        file1.write("\npredictd: "+ str(self.X))
        # file1.close() 

        # self.X_pred = np.reshape(self.X_pred,[8,2])       
        # self.X_correc = np.reshape(X_corrected,[6,2])   # <--------<< Publish 

        # self.X = self.X_correc
        self.prevX = X_corrected#self.X 

    def gainUpdate(self,Sx_k_km1):
        #TODO
        # Write a function to update the Kalman Gain, a.k.a. self.KalGain
        temp = self.covMatrix
        sensorNoise = np.array([[temp[0],0,0,0],[0,temp[7],0,0],[0,0,temp[14],0],[0,0,0,0]])
        C = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

        tempBracket = np.matmul(np.matmul(C,Sx_k_km1),C.transpose()) + sensorNoise
        invTempBracket = inv(tempBracket)
        self.KalGain = np.matmul(Sx_k_km1,(np.matmul(C.transpose(), invTempBracket)))
        return self.KalGain



    
    def dotX(self,x,U):
        # TODO 
        # This is where your motion model should go. The differential equation.
        # This function must be called in the self.predict function to predict the future states.
        x_sensed = x[0] 
        y_sensed = x[1]
        z_sensed = x[2]
        #print(z_sensed)
        theta = np.arctan2(z_sensed, x_sensed)

        X_sensed = [x_sensed, y_sensed, z_sensed, theta]
        omegaL_k, omegaR_k = U
        # diffList = (X_sensed - self.prevX)
        # diffList = [(x-y) for x,y in zip(X_sensed, self.prevX)]
        # for x,y in zip(X_sensed, self.prevX):

        # xDotk = [diff/self.dt for diff in diffList]
        xDotk = (np.subtract(X_sensed, self.prevX)) / self.dt
        #print('xDotk' + str(xDotk))
        # xDotk =  [i/j for i,j in zip(X_sensed - self.prevX, dt)]
        #Differential steering
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
        y_sensed = x[1]
        z_sensed = x[2]

        theta = np.arctan2(z_sensed, x_sensed)
        
        #Modelling the differential steering
        l = 0.5
        R_leg = 0.1
        eta = 8
        dt = self.dt
        front = R_leg/(2*eta)

        Amatrix = np.multiply(np.array([[front*np.cos(theta), front*np.cos(theta), 0, 0],
                                        [front*np.sin(theta), front *np.sin(theta),0, 0],
                                        [0, 0, 1, 0], 
                                        [-R_leg/(l*eta) ,R_leg/(l*eta), 0, 0]]),dt)
        Bmatrix = np.array([[1, 0, 0, 0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        return [Amatrix,Bmatrix]


    def measurement_cb(self, data):
        theta = np.arctan2(data.pose.pose.position.z, data.pose.pose.position.x)
        self.ballloc_xyz = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z,theta]
        self.covMatrix = data.pose.covariance
        #print(self.covMatrix)
        self.X = self.ballloc_xyz
        # print(self.ballloc_xyz)    


if __name__ == '__main__':
    rospy.init_node("EKF")
    rate = rospy.Rate(1)

    # ---------------Define initial conditions --------------- #
    nk = 4       # <------<< Look ahead duration in seconds
    dt = 1       # <------<< Sampling duration of discrete model
    X =  [0,0,0,0]       # <------<< Initial State of the Ball
    U =  [10,10]       # <------<< Initial input to the motion model; U = [omegaL_k, omegaR_k]
    
    filter = EKF(nk,dt,X,U)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
    
        filter.update()     
        rate.sleep()
 