#!/usr/bin/env python

#package imports
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError

#message imports
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo

#subscribed topics
TS_BURGERBOT_PX_WB = "/wafflebot/camera/rgb/burgerbot_px"
TS_BURGERBOT_DIST = "/burgerbot_distance"
TS_BB_DIR = "/wafflebot/camera/rgb/burgerbot_dir"

TS_BURGERBOT_DIST_L = "/burgerbot_distance_lidar"

#published topics
TP_WB_CMDVEL = "/wafflebot/cmd_vel"

#global variables
burgerbot_distance = Float32()
burgerbot_px_WB = Pose()
burgerbot_dir = Pose()
wb_vel = Twist()
#constants
DISTANCE_TOL = 0.05
ANGLE_TOL = 0.05
FOLLOWING_DISTANCE = 0.5

#callbacks
def handleCAM_WB(msg):
    global camInfo_WB
    camInfo_WB = msg

def handleBBPX_WB(msg):
    global burgerbot_px_WB
    burgerbot_px_WB = msg

def handleDistance(msg):
    global burgerbot_distance
    burgerbot_distance = msg

def handleBB_Dir(msg):
    global burgerbot_dir
    burgerbot_dir = msg

#functions
def motion_control():
    #refer navigate_robot example from ELG228 assignment 3
    global wb_vel
    if burgerbot_distance > FOLLOWING_DISTANCE:
        pass

    #rotation control
    #speed control
    pass

def find_angle():
    #find xy orientation of camera
    #get xy portion of unit vector to burgerbot
    #get angle between these 2 vectors
    pass


#node
def wafflebot_driver():
    #init
    rospy.init_node("wafflebot_driver", anonymous = False)
    rate = rospy.Rate(60)

    #subscribers
    rospy.Subscriber(TS_BURGERBOT_PX_WB, Pose, handleBBPX_WB)
    rospy.Subsriber(TS_BURGERBOT_DIST, Float32, handleDistance)
    rospy.Subscriber(TS_BB_DIR, Pose, handleBB_Dir)

    #publishers
    velpub = rospy.Publisher(TP_WB_CMDVEL, Twist, queue_size=10)


if __name__ == '__main__':
    wafflebot_driver()
