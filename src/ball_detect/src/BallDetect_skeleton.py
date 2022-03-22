#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
from math import pi
import roslib
#roslib.load_manifest('my_package')
import cv2
import rospy
import sys
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped

from geometry_msgs.msg import Point
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import pdb
import tf
import tf2_ros
import math
import pcl 
#from ros_numpy.point_cloud2 import get_xyz_points,pointcloud2_to_array
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
#from image_geometry import PinholeCameraModel  


bridge = CvBridge()


class Tracker3D():
    
    center_pixel = (320,240)

    ################
    # <raw_image> : Raw image topic
    # <point_cloud> : POint cloud from depth camera
    ################
    
    def __init__(self,img_topic_name= '/camera/color/image_raw',depth_topic_name= '/camera/depth_registered/points', see_image = False):        
        
        self.image_sub = rospy.Subscriber(img_topic_name,Image,self.image_cb)
        self.depth_sub = rospy.Subscriber(depth_topic_name,PointCloud2,self.depth_cb)
        self.pub = rospy.Publisher("visualization_marker", Marker ,queue_size=100)
        self.pubPWCov = rospy.Publisher("pwcov", PoseWithCovarianceStamped, queue_size=100)
        self.ballloc_pixel = [0,0]
        self.ballloc_xyz = [0,0,0]
        self.ballloc_xyz_copy = [0,0,0]
        self.ball_detected = 0
        self.cv_image = None
        self.depth_image = None
        self.K = []
        self.Rt = np.array([[1,0,0],[0,0,1],[0,-1,-0]])
        self.mask = None

        #defining parameters for covariance
        self.variance_XYZ = [0,0,0]
        self.covarianceMatrix = [0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0] #in centimeters


        self.listener = tf.TransformListener()
        


        # Wait for messages to be published on image and depth topics
        print("Waiting for image and depth topic")
        rospy.wait_for_message(img_topic_name,Image)
        rospy.wait_for_message(depth_topic_name,Image)
        print("-----> Messages received")

        self.rate = rospy.Rate(1)

    def get_depth(self):
        # Function to get depth of the ball

        pass


    def get_xyz(self):
        # Function to compute the x,y,z coordinates of the ball
        xRVIZ = self.ballloc_xyz[2]
        yRVIZ = self.ballloc_xyz[0]*-1
        zRVIZ = self.ballloc_xyz[1] *-1#robots z coordinate due to global X
        self.ballloc_xyz = [xRVIZ,yRVIZ,zRVIZ]
        return self.ballloc_xyz

    def get_x_variance(self):
        return 6.07E-06*self.variance_XYZ[0] + 1.76E-04
    
    def get_y_variance(self):
        return 2.97E-04*self.variance_XYZ[1] + -2.49E-03
    
    def get_XYZ_variance(self):
        #We kno that the error inmeasurement is 2% at 2 metres, we are in centimeter
        xRVIZ = self.ballloc_xyz[2]
        yRVIZ = self.ballloc_xyz[0]*-1
        zRVIZ = self.ballloc_xyz[1] *-1#robots z coordinate due to global X
        self.ballloc_xyz = [xRVIZ,yRVIZ,zRVIZ]
        self.variance_XYZ[2] = ((0.13*self.ballloc_xyz[2])-21)*10
        self.variance_XYZ[0] = self.get_x_variance() #((0.00029*self.ballloc_xyz[0]) - .00254)*10
        self.variance_XYZ[1] = self.get_y_variance() #((0.0324*self.ballloc_xyz[1]) -0.163) #0.0324*x + -0.163
        
        varianceX = self.variance_XYZ[0]
        varianceY = self.variance_XYZ[1]
        varianceZ = self.variance_XYZ[2]
        
        var_xx = varianceX*varianceX
        var_xy = varianceX*varianceY
        var_xz = varianceX*varianceZ

        if np.isnan(var_xx):# var_xx<0.000001:
            var_xx = 0
        if np.isnan(var_xy):# var_xx<0.000001:
            var_xy = 0
        if np.isnan(var_xz):# var_xx<0.000001:
            var_xz = 0

        var_yx = var_xy
        var_yy = varianceY*varianceY
        var_yz = varianceY*varianceZ

        if np.isnan(var_yy):# var_xx<0.000001:
            var_yy = 0
        if np.isnan(var_yz):# var_xx<0.000001:
            var_yz = 0
        
        var_zx = var_xz
        var_zy = var_yz
        var_zz = varianceZ*varianceZ

        # if np.isnan(var_xx):# var_xx<0.000001:
        #     var_xx = 0

        if np.isnan(var_yy):#<0.000001:
            var_yy = 0
        #X,Y,Z
        self.covarianceMatrix = [var_xx,var_xy,var_xz,0,0,0,
                            var_yx,var_yy,var_yz,0,0,0,
                            var_zx,var_zy,var_zz,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0,
                            0,0,0,0,0,0]
        print(self.covarianceMatrix)

        return self.covarianceMatrix, self.ballloc_xyz
        #return (self.variance_XYZ)


    def pub_viz(self): #ball with respect to camera. Other python file ball with respect to world.
        # Publish the marker for the ball
        if self.detected ==1:
            xRVIZ = self.ballloc_xyz[2]
            yRVIZ = self.ballloc_xyz[0]*-1
            zRVIZ = self.ballloc_xyz[1] *-1
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.Time.now()
            marker.id = 2
            marker.type = marker.SPHERE
            marker.action =marker.ADD
            marker.pose.position.x = xRVIZ
            marker.pose.position.y = yRVIZ
            marker.pose.position.z = zRVIZ
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            marker.scale.x = .1
            marker.scale.y = .1
            marker.scale.z = .1
            marker.color.a = 1.0 
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            self.pub.publish(marker)
        pass

    def posewithCovar(self):
        # Function to get depth of the ball
        pwc = PoseWithCovarianceStamped()
        pwc.header.frame_id = "camera_link"
        pwc.header.stamp = rospy.Time.now()
        pwc.pose.pose.position.x = self.ballloc_xyz[0]
        pwc.pose.pose.position.y = self.ballloc_xyz[1]
        pwc.pose.pose.position.z = self.ballloc_xyz[2]
        pwc.pose.pose.orientation.x = 0
        pwc.pose.pose.orientation.y = 0
        pwc.pose.pose.orientation.z = 0
        pwc.pose.pose.orientation.w = 1
        pwc.pose.covariance = self.covarianceMatrix
        #self.covarianceMatrix

        # pwc.pose.covariance = self.covarianceMatrix

        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = 2
        marker.type = marker.SPHERE
        marker.action =marker.ADD
        marker.pose.position.x = self.ballloc_xyz[0]
        marker.pose.position.y = self.ballloc_xyz[1]
        marker.pose.position.z = self.ballloc_xyz[2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = .1
        marker.scale.y = .1
        marker.scale.z = .1
        marker.color.a = 1.0 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.pub.publish(marker)
        self.pubPWCov.publish(pwc)

        
        pass
   

    def image_cb(self,data):
        try:
		    self.cv_image = bridge.imgmsg_to_cv2(data,"bgr8") #receives image
        except CvBridgeError as e:
            print(e)  #prints error
        #print('here')
        cv_image_hsv = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2HSV) #converts image from BGR to HSV
	    

	    # Define lower and upper range of the colors to generate a mask using hsv #(242,21,65), (257,45,100)
        #cv2.imshow('ImageWindow', cv_image_hsv)
        MIN = np.array([110,30,60])
        MAX = np.array([150,205,205])
        # MIN = np.array([90,200,0])
        # MAX = np.array([100,255,255])

        mask = cv2.inRange(cv_image_hsv, MIN, MAX) # 

        # Process your mask to reduce noise
        self.mask = mask
        masked = cv2.bitwise_and(self.cv_image, self.cv_image, mask=self.mask)
        # cv2.waitKey(1)
        # cv2.imshow('mask',mask)

	    # find contours in the mask and initialize the current
	    # (x, y) center of the ball and ublish the (x,y) pixel coordinates of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	    	cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        #print(cnts)
        #cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        #figure out how to determine the biggest countor and return its center
        if len(cnts) != 0:
            # find th e biggest countour (c) by the area
            cntsSorted = sorted(cnts, key=lambda x: cv2.contourArea(x))
            index = len(cntsSorted)-1
            c = max(cnts, key = cv2.contourArea)
            largest = int(cv2.contourArea(cntsSorted[index]))
            radius = int(math.sqrt(largest/pi))
            #print(radius)
            if radius > 15:
                self.ball_detected =1
            else:
                self.ball_detected =0
            M = cv2.moments(c)
            # print(int(M["m00"]))

            if(int(M["m00"])!=0):
                #trying to implment a basic filter
                tempX = int(M["m10"] / M["m00"])
                tempY = int(M["m01"] / M["m00"])
                
                # minDiff = 10
                # if(abs(self.ballloc_pixel[0])>2 and abs(self.ballloc_pixel[1])>2):
                #     if(abs(tempX - self.ballloc_pixel[0])>minDiff):
                #         print("tempX: " + str(tempX))
                #         tempX = self.ballloc_pixel[0]
                #     if(abs(tempY - self.ballloc_pixel[1])>minDiff):
                #         print("tempY: " + str(tempY))
                #         tempY = self.ballloc_pixel[1]

                # self.ballloc_pixel =(tempX, tempY)
                self.ballloc_pixel = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.circle(self.cv_image, self.ballloc_pixel, radius, (0, 0, 255), 3)
                #cv2.imshow('ImageWindow', self.cv_image)

                self.ballloc_xyz_copy[2] = round(self.ballloc_xyz[2]*100, 2)
                self.ballloc_xyz_copy[0]  = round(self.ballloc_xyz[0]*100, 2)
                self.ballloc_xyz_copy[1]  = -1* round(self.ballloc_xyz[1]*100, 2)
                #print(self.ballloc_xyz_copy[1])
                #self.get_xyz()
                
                cv2.putText(self.cv_image, str((self.ballloc_xyz_copy)), (int(self.ballloc_pixel[0]),int(self.ballloc_pixel[1])),cv2.FONT_HERSHEY_SIMPLEX, .75, (255, 0, 0),2)
        cv2.waitKey(1)
        cv2.imshow('ImageWindow', self.cv_image)

	    # only proceed if at least one contour was found
    
        

    def depth_cb(self,data):
        self.depth_image = data
        twoDX = self.ballloc_pixel[0] 
        twoDY = self.ballloc_pixel[1] 
        #print((twoDX))
        #print((twoDY))
        #grab images from pointfield
        gen = point_cloud2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True, uvs=[[int(twoDX),int(twoDY)]])
        for p in gen:
            #print(len(p))
            x =p[0]
            y =p[1]
            z =p[2]
            self.ballloc_xyz =[x,y,z]


    

if __name__ == "__main__":
    rospy.init_node("measure_3d")
    tracker = Tracker3D()
    viz_img = True
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # Publish the (x,y) coordinates of the ball in the pixel coordinates
        #tracker.get_xyz()
        #tracker.pub_viz()
        #print(tracker.get_covariance_matrix())
        tracker.get_XYZ_variance()
        if tracker.ball_detected ==1:
            tracker.posewithCovar()
        #call pub_tf
        rate.sleep()