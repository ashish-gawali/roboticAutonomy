import numpy as np
import human_walk
import matplotlib.pyplot as plt
import pdb
import rospy
from geometry_msgs.msg import PoseWithCovariance
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

        self.measurement_sub = rospy.Subscriber("/pwcov",PoseWithCovariance,self.measurement_cb) # <--------<< Subscribe to the ball pose topic

        print(self.z_k)

        self.Sx_k_k = self.Sigma_init



    def prediction(self,x,U,Sigma_km1_km1):
        #TODO 
        # Use the motion model and input sequence to predict a n step look ahead trajectory. 
        # You will use only the first state of the sequence for the rest of the filtering process.
        # So at a time k, you should have a list, X = [xk, xk_1, xk_2, xk_3, ..., xk_n] and you will use only xk. 
        # The next time this function is called a lnew list is formed.
        for i in range( self.nk):
            X[i] = x + (self.dotX(x,U)*self.dt*i)
        
        return X 
        


    def correction(self,x_predict, Sx_k_km1, z_k, KalGain):
        #TODO
        # Write a function to correct your prediction using the observed state.


    def update(self):
        self.X_pred = self.X 
        
        X_predicted,Sx_k_km1, A = self.prediction(self.X,self.U,self.Sx_k_k)                        # PREDICTION STEP  
        X_corrected, self.Sx_k_k = self.correction(X_predicted, Sx_k_km1, self.z_k, self.KalGain)   # CORRECTION STEP 
        self.gainUpdate(Sx_k_km1)                                                                   # GAIN UPDATE       
        self.X = X_corrected  

        self.X_pred = np.reshape(self.X_pred,[6,2])       
        self.X_correc = np.reshape(self.X_correc,[6,2])   # <--------<< Publish 

        self.X = self.X_correc
        self.prevX = self.X 

    def gainUpdate(self,Sx_k_km1):
        #TODO
        # Write a function to update the Kalman Gain, a.k.a. self.KalGain

    
    def dotX(self,x,U):
        # TODO 
        # This is where your motion model should go. The differential equation.
        # This function must be called in the self.predict function to predict the future states.
        x_sensed = x[0] 
        y_sensed = x[1]
        z_sensed = x[2]

        theta = np.arctan2(z_sensed, x_sensed)

        X_sensed = [x_sensed, y_sensed, z_sensed, theta]

        omegaL_k, omegaR_k = U
        xDotk = (X_sensed - self.prevX) / self.dt 
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

        Amatrix = np.multiply(np.array([[front*np.cos(theta), front*np.cos(theta), 0], [front*np.sin(theta), front *np.sin(theta),0], [0, 0, 1], [-R_leg/(l*eta) ,R_leg/(l*eta), 0]]),dt)
        Bmatrix = np.array([[1, 0, 0, 0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
        return [Amatrix,Bmatrix]


    def measurement_cb(self, data):
        self.ballloc_xyz = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
        self.X = self.ballloc_xyz
        # print(self.ballloc_xyz)    


if __name__ == '__main__':
    rospy.init_node("EKF")
    rate = rospy.Rate(1)

    # ---------------Define initial conditions --------------- #
    nk = None       # <------<< Look ahead duration in seconds
    dt = None       # <------<< Sampling duration of discrete model
    X =  None       # <------<< Initial State of the Ball
    U =  None       # <------<< Initial input to the motion model; U = [omegaL_k, omegaR_k]
    
    filter = EKF(nk,dt,X,U)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
    
        filter.update()     
        rate.sleep()
 