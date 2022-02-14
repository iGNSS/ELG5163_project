#!/usr/bin/env python
"""
This code is part of the displaced_stereo_vision project for ELG 5163 Machine 
Vision.

The objective of this node is to calculate the 3D unit vectors between the 
quadcopter and burgerbot, and between the wafflebot and burgerbot using the 
pixel location of their respective cameras. The unit vectors are then published 
at a rate of 10 Hz.

Created on Feb 11 2022

@author: Mathieu Falardeau
"""

###################    main library imports    ################################
from __future__ import print_function

import roslib
roslib.load_manifest('displaced_stereo_vision')
import sys
import rospy
import tf
import image_geometry as ig

from geometry_msgs.msg import Pose, Pose2D
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_inverse as qi, quaternion_multiply as qm

CAMERA_INFO_WB_SUB_TOPIC = "/wafflebot/camera/rgb/camera_info"
CAMERA_INFO_QC_SUB_TOPIC = "/quadcopter/camera/camera_info"
#IMAGE_CENTER_QC_SUB_TOPIC = "/wafflebot/"
#IMAGE_CENTER_WB_SUB_TOPIC = "/quadcopter/"

###################   Class   #################################################

class direction_calculator:

    def __init__(self):
    
        rospy.init_node('direction_calculator', anonymous=False)

        self.listener = tf.TransformListener()
        #self.qc_point = Pose2D()  
        self.wb_point = Pose2D()  
        
        #self.qc_pcm = ig.PinholeCameraModel()
        self.wb_pcm = ig.PinholeCameraModel()
        
        #self.qc2bb = Pose()
        self.wb2bb = Pose()
        
        #self.qc2bb_pub = rospy.Publisher("quad_2_burger_uv",Pose)
        #self.wb2bb_pub = rospy.Publisher("waffle_2_burger_uv",Pose)

        #self.qc_point_sub = rospy.Subscriber(IMAGE_CENTER_QC_SUB_TOPIC,Pose2D,self.update_qc_point)
        #self.wb_point_sub = rospy.Subscriber(IMAGE_CENTER_WB_SUB_TOPIC,Pose2D,self.update_wb_point)
        #self.qc_pcm_sub = rospy.Subscriber(CAMERA_INFO_QC_SUB_TOPIC,CameraInfo,self.update_qc_pcm)
        self.wb_pcm_sub = rospy.Subscriber(CAMERA_INFO_WB_SUB_TOPIC,CameraInfo,self.update_wb_pcm)

    def update_qc_point(self, rec_msg):
        self.qc_point = rec_msg
        
    def update_wb_point(self, rec_msg):
        self.wb_point = rec_msg
            
    def update_qc_pcm(self, rec_msg):
        
        self.qc_pcm.fromCameraInfo(rec_msg)
        
    def update_wb_pcm(self, rec_msg):
        
        self.wb_pcm.fromCameraInfo(rec_msg)
        
    def calc_uv(self, raw_coord, pcm, odom_topic):
        try:
            rec_coord = pcm.rectifyPoint(raw_coord)
            uv_cam = pcm.projectPixelTo3dRay(rec_coord)
        except: 
            pass
        try:
            (trans,rot) = self.listener.lookupTransform(self.wb_pcm.tfFrame(), odom_topic, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
            
        uv_world = qm((qm(rot,(uv_cam+(0,)))),qi(rot))

        return uv_world[:3]
        

    def update_direction_calculator(self):
        
        r = rospy.Rate(2) # Rate of 5 Hz
        
        #self.listener.waitForTransform(self.wb_pcm.tfFrame(), '/wafflebot_tf/odom', rospy.Time(), rospy.Duration(4.0))
        #listener.waitForTransform(self.qc_pcm.tfFrame(), '/quadcopter_tf/odom', rospy.Time(), rospy.Duration(4.0))
        
        while not rospy.is_shutdown():
            #self.wb_point.x = self.wb_pcm.cx()
            #self.wb_point.y = self.wb_pcm.cy()
            print(self.wb_pcm.tfFrame())
            #self.qc2bb = calc_uv((self.qc_point.x,self.qc_point.y),self.qc_pcm)
            #world_wb_uv = self.calc_uv((self.wb_point.x,self.wb_point.y),self.wb_pcm,'/wafflebot_tf/odom')
            # print("\nThe world vector is pointing at : \n\tvector x: %3.3f\n\tvector y: %3.3f\n\tvector z: %3.3f", 
            #     world_wb_uv[0],world_wb_uv[1],world_wb_uv[2]) 
            # #self.qc2bb_pub.publish(self.qc2bb)
            # #self.wb2bb_pub.publish(self.wb2bb)
            
            # self.wb2bb.orientation.x = world_wb_uv[0]
            # self.wb2bb.orientation.y = world_wb_uv[1]
            # self.wb2bb.orientation.z = world_wb_uv[2]


            r.sleep()

def main(args):
    dc = direction_calculator()
  
    try:
        dc.update_direction_calculator()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)

