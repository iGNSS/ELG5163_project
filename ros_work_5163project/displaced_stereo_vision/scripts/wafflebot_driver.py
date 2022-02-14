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


#subscribed topics
TS_BURGERBOT_PX_WB = "/wafflebot/camera/rgb/burgerbot_px"
TS_WB_ODO = "/wafflebot/odom"
TS_BURGERBOT_DIST = "/burgerbot_distance"

#published topics
TP_WB_CMDVEL = "/wafflebot/cmd_vel"

#global variables
burgerbot_distance = Float32()
burgerbot_px_WB = Pose()

#constants
DISTANCE_TOL = 0.05
ANGLE_TOL = 0.05

#functions
def HandleDistance(msg):
    global burgerbot_distance
    burgerbot_distance = msg
def HandleBBPX_WB(msg):
    global burgerbot_px_WB
    burgerbot_px_WB = msg

#node
def wafflebot_driver():
    #init
    rospy.init_node("wafflebot_driver", anonymous = False)
    rate = rospy.Rate(60)

if __name__ == '__main__':
    wafflebot_driver()
