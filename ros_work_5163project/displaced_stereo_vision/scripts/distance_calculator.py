#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

#subscribed topics
TS_BURGERBOT_PX_QC = "/quadcopter/front_cam/rgb/burgerbot_px"
TS_BURGERBOT_PX_WB = "/wafflebot/camera/rgb/burgerbot_px"
TS_WB_CAMINFO = "/wafflebot/camera/rgb/camera_info"
TS_QC_CAMINFO = "/quadcopter/front_cam/camera/camera_info"

#published topics
TP_BURGERBOT_DIST = "burgerbot_distance"

burgerbot_px_QC = Pose()
burgerbot_px_WB = Pose()

def HandleBBPX_QC(msg):
    global burgerbot_px_QC
    burgerbot_px_QC = msg

def HandleBBPX_WB(msg):
    global burgerbot_px_WB
    burgerbot_px_WB = msg

def distance_calculator():
    rospy.init_node("distance_calculator", anonymous = False)
    rate = rospy.Rate(60)

    rospy.Subscriber(TS_BURGERBOT_PX_QC, Pose, HandleBBPX_QC)
    rospy.Subscriber(TS_BURGERBOT_PX_WB, Pose, HandleBBPX_WB)
