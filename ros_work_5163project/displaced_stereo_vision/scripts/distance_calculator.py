#!/usr/bin/env python

#package imports
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError

#message imports
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32

#subscribed topics
TS_BURGERBOT_PX_QC = "/quadcopter/front_cam/rgb/burgerbot_px"
TS_BURGERBOT_PX_WB = "/wafflebot/camera/rgb/burgerbot_px"
TS_QC_CAMINFO = "/quadcopter/front_cam/camera/camera_info"
TS_WB_CAMINFO = "/wafflebot/camera/rgb/camera_info"
TS_QC_ODO = "/quadcopter/ground_truth/state"
TS_WB_ODO = "/wafflebot/odom"


#published topics
TP_BURGERBOT_DIST = "/burgerbot_distance"

#global variables
burgerbot_px_QC = Pose()
burgerbot_px_WB = Pose()
camInfo_QC = CameraInfo()
camInfo_WB = CameraInfo()
QC_cam = PinholeCameraModel()
WB_cam = PinholeCameraModel()
QC_Odo = Odometry()
WB_Odo = Odometry()
burgetbot_distance = Float32()
#constants
#HT quadcopter to quadcopter cam
#HT wafflebot to wafflebot cam

#callback functions
def HandleBBPX_QC(msg):
    global burgerbot_px_QC
    burgerbot_px_QC = msg

def HandleBBPX_WB(msg):
    global burgerbot_px_WB
    burgerbot_px_WB = msg

def handleCAM_QC(msg):
    global camInfo_QC
    global QC_CAM
    camInfo_QC = msg
    QC_cam.fromCameraInfo(msg)

def handleCAM_WB(msg):
    global camInfo_WB
    global WB_CAM
    camInfo_WB = msg
    WB_cam.fromCameraInfo(msg)

#need to work with odometry
def handleOdo_QC(msg):
    global QC_Odo
    QC_Odo = msg

def handleOdo_WB(msg):
    global WB_Odo
    WB_Odo = msg

#node
def distance_calculator():
    #init node
    rospy.init_node("distance_calculator", anonymous = False)
    rate = rospy.Rate(60)
    #subscribers
    rospy.Subscriber(TS_BURGERBOT_PX_QC, Pose, HandleBBPX_QC)
    rospy.Subscriber(TS_BURGERBOT_PX_WB, Pose, HandleBBPX_WB)
    rospy.Subscriber(TS_QC_CAMINFO, CameraInfo, handleCAM_QC)
    rospy.Subscriber(TS_WB_CAMINFO, CameraInfo, handleCAM_WB)
    rospy.Subscriber(TS_QC_ODO, Odometry, handleOdo_QC)
    rospy.Subscriber(TS_QC_ODO, Odometry, handleOdo_WB)

    #publishers

    while not rospy.is_shutdown():
        print(QC_cam.tf_frame)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    distance_calculator()
