#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
ball_detected = 0
def get_x_variance(test):
    temp = 6.07E-06*test + 1.76E-04
    return abs(temp)

def get_y_variance(test):
    temp = 2.97E-04*test + -2.49E-03
    return abs(temp)

def get_z_variance(test):
    temp = ((0.013*test)-0.0021)
    return abs(temp)

def get_XYZ_variance(trans):
    #We kno that the error inmeasurement is 2% at 2 metres, we are in centimeter
    xPlace = trans.transform.translation.x
    yPlace = trans.transform.translation.y
    zPlace = trans.transform.translation.z #robots z coordinate due to global X
    #self.variance_XYZ[2] = ((13*zRVIZ)-21)
    variance_XYZ= [0,0,0]
    variance_XYZ[2] = ((0.013*zPlace)-0.0021)
    variance_XYZ[0] = get_x_variance(xPlace) #((0.00029*self.ballloc_xyz[0]) - .00254)*10
    variance_XYZ[1] = get_y_variance(yPlace) #((0.0324*self.ballloc_xyz[1]) -0.163) #0.0324*x + -0.163
    
    varianceX = variance_XYZ[0]
    varianceY = variance_XYZ[1]
    varianceZ = variance_XYZ[2]

    varianceXRVIZ = variance_XYZ[2]
    varianceYRVIZ = variance_XYZ[0]
    varianceZRVIZ = variance_XYZ[1]
    
    covarianceMatrix = [varianceX,0 ,0 ,0,0,0,
                        0,varianceY,0,0,0,0,
                        0,0,varianceZ,0,0,0,
                        0,0,0,0,0,0,
                        0,0,0,0,0,0,
                        0,0,0,0,0,0]
    #print(self.covarianceMatrix)

    return covarianceMatrix
    #return (self.variance_XYZ)
if __name__ == "__main__":
    rospy.init_node("node1")
    #rospy.init_node('ball_wrt_world')
    vis_pub = rospy.Publisher("visualization_marker", Marker ,queue_size=10)
    pubPWCov = rospy.Publisher("pwcov", PoseWithCovarianceStamped, queue_size=100)

    tfbuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfbuffer)

    rate = rospy.Rate(20)
    frame = 'ball_wrt_cam_frame'
    while not rospy.is_shutdown():
        try:
            (trans) = tfbuffer.lookup_transform('map', frame, rospy.Time()) #make 2d goal out of trans and rot
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.id = 2
            marker.type = marker.SPHERE
            marker.action =marker.ADD
            marker.pose.position.x = trans.transform.translation.x
            marker.pose.position.y = trans.transform.translation.y
            marker.pose.position.z = trans.transform.translation.z
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
            # if (trans.transform.translation.x <5 and trans2.transform.translation.x< 5) and (trans.transform.translation.y > -3 and trans2.transform.translation.y> -3):
            #     ball_detected = 1
            vis_pub.publish(marker)
            # else:
            #     ball_detected = 0
            #     pass

            pwc = PoseWithCovarianceStamped()
            pwc.header.frame_id = "map"
            pwc.header.stamp = rospy.Time.now()
            pwc.pose.pose.position.x = trans.transform.translation.x
            pwc.pose.pose.position.y = trans.transform.translation.y
            pwc.pose.pose.position.z = trans.transform.translation.z
            pwc.pose.pose.orientation.x = 0
            pwc.pose.pose.orientation.y = 0
            pwc.pose.pose.orientation.z = 0
            pwc.pose.pose.orientation.w = 1
            pwc.pose.covariance = get_XYZ_variance(trans)
            pubPWCov.publish(pwc)

            #print(self.covarianceMatrix)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    
        rate.sleep()