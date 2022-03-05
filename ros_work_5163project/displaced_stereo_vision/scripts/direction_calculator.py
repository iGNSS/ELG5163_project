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
import numpy as np

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_conjugate as qc, quaternion_multiply as qm

CAMERA_INFO_WB_SUB_TOPIC = "/wafflebot/camera/rgb/camera_info"
CAMERA_INFO_QC_SUB_TOPIC = "/quadcopter/front_cam/camera/camera_info"
BURGERBOT_TRACKER_WB_SUB_TOPIC = "/wafflebot/camera/rgb/burgerbot_px"
BURGERBOT_TRACKER_QC_SUB_TOPIC = "/quadcopter/front_cam/rgb/burgerbot_px"

BURGERBOT_DIRECTION_WB_PUB_TOPIC = "/wafflebot/camera/rgb/burgerbot_dir"
BURGERBOT_DIRECTION_QC_PUB_TOPIC = "/quadcopter/front_cam/rgb/burgerbot_dir"

###################   Class   #################################################

class direction_calculator:

    def __init__(self):
    
        rospy.init_node('direction_calculator', anonymous=False)

        self.listener = tf.TransformListener()
        self.qc_point = Pose()
        self.wb_point = Pose()
        
        self.qc_pcm = ig.PinholeCameraModel()
        self.wb_pcm = ig.PinholeCameraModel()
        
        self.qc2bb = Pose()
        self.wb2bb = Pose()
        
        self.qc2bb_pub = rospy.Publisher(BURGERBOT_DIRECTION_QC_PUB_TOPIC,Pose,queue_size=10)
        self.wb2bb_pub = rospy.Publisher(BURGERBOT_DIRECTION_WB_PUB_TOPIC,Pose,queue_size=10)

        self.wb_pcm_sub = rospy.Subscriber(CAMERA_INFO_WB_SUB_TOPIC,CameraInfo,self.update_wb_pcm)
        self.wb_point_sub = rospy.Subscriber(BURGERBOT_TRACKER_WB_SUB_TOPIC,Pose,self.update_bb_tracker_wb)
        
        self.qc_pcm_sub = rospy.Subscriber(CAMERA_INFO_QC_SUB_TOPIC,CameraInfo,self.update_qc_pcm)
        self.qc_point_sub = rospy.Subscriber(BURGERBOT_TRACKER_QC_SUB_TOPIC,Pose,self.update_bb_tracker_qc)

    def update_bb_tracker_qc(self, rec_msg):
        self.qc_point = rec_msg
        
    def update_bb_tracker_wb(self, rec_msg):
        self.wb_point = rec_msg
            
    def update_qc_pcm(self, rec_msg):
        self.qc_pcm.fromCameraInfo(rec_msg)
        
    def update_wb_pcm(self, rec_msg):
        self.wb_pcm.fromCameraInfo(rec_msg)
        
    def calc_uv(self, raw_coord, pcm, odom_topic, camera_frame_topic):
        uv_world = np.empty(6)
        uv_world.fill(np.nan)

        try:
            rec_coord = pcm.rectifyPoint(raw_coord)
            rospy.loginfo("\nThe camera raw coord : \n\trow : %3.3f\n\tcol : %3.3f\nThe camera rec coord : \n\trow : %3.3f\n\tcol : %3.3f",
                          raw_coord[0],raw_coord[1],rec_coord[0],rec_coord[1])
            uv_cam = pcm.projectPixelTo3dRay(rec_coord)
            rospy.loginfo("\nThe camera 3D ray : \n\tx : %3.3f\n\ty : %3.3f\n\tz : %3.3f\n",
                          uv_cam[0],uv_cam[1],uv_cam[2])
        except: 
            pass

        try:
            print(camera_frame_topic)
            (trans,rot) = self.listener.lookupTransform(odom_topic, camera_frame_topic, rospy.Time(0))
            position = np.asarray(trans)
            a = uv_cam+(0,)
            b = qm(rot,a)
            c = qc(rot)
            d = qm(b,c)
            orientation = d[0:3]
            uv_world = np.append(position,orientation)

            return uv_world
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            return uv_world

    def update_direction_calculator(self):
        
        r = rospy.Rate(10) # Rate of 10 Hz
        
        #self.listener.waitForTransform(self.wb_pcm.tfFrame(), '/wafflebot_tf/odom', rospy.Time(), rospy.Duration(4.0))
        #listener.waitForTransform(self.qc_pcm.tfFrame(), '/quadcopter_tf/odom', rospy.Time(), rospy.Duration(4.0))
        
        while not rospy.is_shutdown():
            if (not self.wb_pcm.tfFrame()) or (not self.wb_point.position.x) or (not self.qc_pcm.tfFrame()) or \
                    (not self.qc_point.position.x) :
                if not self.wb_pcm.tfFrame():
                    print("No Wafflebot camera frame data")

                if not self.wb_point.position.x:
                    print("No Wafflebot image position data")
                    rospy.loginfo(self.wb_point)

                if not self.qc_pcm.tfFrame():
                    print("No Quadcopter camera frame data")

                if not self.qc_point.position.x:
                    print("No Quadcopter image position data")
                    rospy.loginfo(self.wb_point)
            else:
                #self.qc2bb = calc_uv((self.qc_point.x,self.qc_point.y),self.qc_pcm)
                world_qc_uv = self.calc_uv((self.qc_point.position.x,self.qc_point.position.y),self.qc_pcm,'/world',"/front_cam_optical_frame")
                world_wb_uv = self.calc_uv((self.wb_point.position.x,self.wb_point.position.y),self.wb_pcm,'/wafflebot_tf/odom',self.wb_pcm.tfFrame())

                self.wb2bb.position.x = world_wb_uv[0]
                self.wb2bb.position.y = world_wb_uv[1]
                self.wb2bb.position.z = world_wb_uv[2]
                self.wb2bb.orientation.x = world_wb_uv[3]
                self.wb2bb.orientation.y = world_wb_uv[4]
                self.wb2bb.orientation.z = world_wb_uv[5]

                self.qc2bb.position.x = world_qc_uv[0]
                self.qc2bb.position.y = world_qc_uv[1]
                self.qc2bb.position.z = world_qc_uv[2]
                self.qc2bb.orientation.x = world_qc_uv[3]
                self.qc2bb.orientation.y = world_qc_uv[4]
                self.qc2bb.orientation.z = world_qc_uv[5]

                print("Wafflebot Info")
                rospy.loginfo(self.wb2bb)
                print("Quadcopter Info")
                rospy.loginfo(self.qc2bb)

                self.qc2bb_pub.publish(self.qc2bb)
                self.wb2bb_pub.publish(self.wb2bb)

            r.sleep()

def main(args):
    dc = direction_calculator()
  
    try:
        dc.update_direction_calculator()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)

