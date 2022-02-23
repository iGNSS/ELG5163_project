#!/usr/bin/env python

#package imports
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import image_geometry as ig

#message imports
from geometry_msgs.msg import Pose, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo

#subscribed topics
TS_BURGERBOT_DIST = "/burgerbot_distance"
TS_WB_ODO = "/wafflebot/odom"
TS_BURGERBOT_DIST_L = "/burgerbot_distance_lidar"

TS_BURGERBOT_PX_WB = "/wafflebot/camera/rgb/burgerbot_px"

TS_WB_CAM_INFO = "/wafflebot/camera/rgb/camera_info"
#published topics
TP_WB_CMDVEL = "/wafflebot/cmd_vel"

#global variables


burgerbot_px = Pose()
wb_vel = Twist()
WB_Cam_info = CameraInfo()
WB_CAM = ig.PinholeCameraModel()

sensed_object = Pose2D()

burgerbot_distance = 0
angle2BB = 0

#constants
DISTANCE_TOL = 0.05
ANGLE_TOL = 0.05
FOLLOWING_DISTANCE = 0.5

#callbacks
def handleDistance(msg):
    global burgerbot_distance
    burgerbot_distance = msg.position.x

def handleBB_Dir(msg):
    global burgerbot_dir
    burgerbot_dir = msg

def handleBB_PX(msg):
    global burgerbot_px
    burgerbot_px = msg

def Process_WB_CAM(msg):
    global WB_Cam_info
    global WB_CAM
    global angle2BB

    WB_Cam_info = msg
    WB_CAM.fromCameraInfo(msg)
    raw_coord = (burgerbot_px.position.x, burgerbot_px.position.y)
    rec_coord = WB_CAM.rectifyPoint(raw_coord)
    vec2BB = WB_CAM.projectPixelTo3dRay(rec_coord)
    angle2BB = np.arctan2(vec2BB[0], vec2BB[2])

#functions
def motion_control():
    #refer navigate_robot example from ELG228 assignment 3
    global wb_vel

    if abs(angle2BB) > ANGLE_TOL:
        wb_vel.angular.z = -angle2BB
    else:
        wb_vel.angular.z = 0

    if burgerbot_distance - FOLLOWING_DISTANCE > DISTANCE_TOL:
        wb_vel.linear.x = burgerbot_distance - FOLLOWING_DISTANCE
    else:
        wb_vel.linear.x = 0

#node
def wafflebot_driver():
    #init
    rospy.init_node("wafflebot_driver", anonymous = False)
    rate = rospy.Rate(20)

    #subscribers
    rospy.Subscriber(TS_BURGERBOT_DIST, Pose, handleDistance)
    rospy.Subscriber(TS_BURGERBOT_PX_WB, Pose, handleBB_PX)
    rospy.Subscriber(TS_WB_CAM_INFO, CameraInfo, Process_WB_CAM)

    #publishers
    velpub = rospy.Publisher(TP_WB_CMDVEL, Twist, queue_size=10)

    while not rospy.is_shutdown():

        motion_control()
        velpub.publish(wb_vel)
        print(wb_vel)
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    wafflebot_driver()
