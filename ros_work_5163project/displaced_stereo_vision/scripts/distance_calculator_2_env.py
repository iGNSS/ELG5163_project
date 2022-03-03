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
from std_msgs.msg import Float64
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_conjugate as qc, quaternion_multiply as qm

BURGERBOT_DIRECTION_WB_SUB_TOPIC = "/wafflebot/camera/rgb/burgerbot_dir"
BURGERBOT_DIRECTION_QC_SUB_TOPIC = "/quadcopter/front_cam/rgb/burgerbot_dir"

BURGERBOT_DISTANCE_WB_PUB_TOPIC = "/wafflebot/camera/rgb/burgerbot_distance"

###################   Class   #################################################

class distance_calculator:

    def __init__(self):

        rospy.init_node('distance_calculator_2', anonymous=False)

        self.wb2bb = Pose()
        self.qc2bb = Pose()
        self.wb2bb_dist = Pose()

        self.wb2bb_dist_pub = rospy.Publisher(BURGERBOT_DISTANCE_WB_PUB_TOPIC,Pose,queue_size=10)

        self.wb2bb_sub = rospy.Subscriber(BURGERBOT_DIRECTION_WB_SUB_TOPIC,Pose,self.update_bb_direction_wb)
        self.qc2bb_sub = rospy.Subscriber(BURGERBOT_DIRECTION_QC_SUB_TOPIC,Pose,self.update_bb_direction_qc)

    def update_bb_direction_wb(self, rec_msg):
        self.wb2bb = rec_msg

    def update_bb_direction_qc(self, rec_msg):
        self.qc2bb = rec_msg

    def calc_dist(self, qc_pos, wb_pos, qc2bb_vec, wb2bb_vec):
        uv = lambda a : a/np.linalg.norm(a)
        angle = lambda a,b : np.arccos(np.dot(uv(a),uv(b)))
        dist = lambda a : np.linalg.norm(a)

        qc2wb_vec = wb_pos - qc_pos
        wb2qc_vec = -qc2wb_vec

        qc_angle = angle(qc2wb_vec,qc2bb_vec)
        wb_angle = angle(wb2qc_vec,wb2bb_vec)

        bb_angle = np.pi - (qc_angle + wb_angle)

        wb2bb_dist = (dist(qc2wb_vec)*np.sin(qc_angle))/np.sin(bb_angle)

        return wb2bb_dist


    def update_distance_calculator(self):

        r = rospy.Rate(5) # Rate of 5 Hz

        while not rospy.is_shutdown():
            if (not self.wb2bb.position.x) or (not self.qc2bb.position.x) :
                if not self.wb2bb.position.x:
                    print("No Wafflebot direction data")

                if not self.qc2bb.position.x:
                    print("No Quadcopter direction data")
 

            else:

                wb_pos = np.array([self.wb2bb.position.x,self.wb2bb.position.y,self.wb2bb.position.z])
                qc_pos = np.array([self.qc2bb.position.x,self.qc2bb.position.y,self.qc2bb.position.z])

                wb2bb_vec = np.array([self.wb2bb.orientation.x,self.wb2bb.orientation.y,self.wb2bb.orientation.z])
                qc2bb_vec = np.array([self.qc2bb.orientation.x,self.qc2bb.orientation.y,self.qc2bb.orientation.z])

                self.wb2bb_dist.position.x = self.calc_dist(qc_pos, wb_pos, qc2bb_vec, wb2bb_vec)

                rospy.loginfo("Distance between Wafflebot and Burgerbot (m): %3.3f",wb2bb_dist.position.x)

                self.wb2bb_dist_pub.publish(self.wb2bb_dist)

            r.sleep()

def main(args):
    dc = distance_calculator()

    try:
        dc.update_distance_calculator()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)
