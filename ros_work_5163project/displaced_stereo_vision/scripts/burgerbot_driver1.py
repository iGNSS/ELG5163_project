#!/usr/bin/env python

#package imports
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import image_geometry as ig

#message imports
from geometry_msgs.msg import Pose, Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo

#subscribed topics
TS_BURGERBOT_ODOM = "/burgerbot/odom"

#publish topics
TP_BB_CMDVEL = "/burgerbot/cmd_vel"

#global variables
TARGET_LIST =[[2,0], [4,0]]

current_target = Pose2D()
current_target.x = TARGET_LIST[0][0]
current_target.y = TARGET_LIST[0][1]
delta_pose = Pose2D()
BB_pose = Pose2D()

BB_vel = Twist()


#constants
DISTANCE_TOL = 0.05
ANGLE_TOL = 0.05


#functions

#changes target position
def target_selector():
    global TARGET_LIST
    global current_target
    global BB_pose
    i = 0

    current_target = TARGET_LIST[i]

    for i in range(len(TARGET_LIST)):
        if np.sqrt(delta_pose.x**2 + delta_pose.y**2) < DISTANCE_TOL:
            i = i +1
            current_target.x = TARGET_LIST[i][0]
            current_target.y = TARGET_LIST[i][1]
            print(current_target)


def calculate_delta_pose():
    global delta_pose
    delta_pose.x = current_target.x - BB_pose.x
    delta_pose.y = current_target.y - BB_pose.y
    target_angle = np.arctan2(delta_pose.y, delta_pose.x)
    delta_pose.theta = target_angle - BB_pose.theta

def motion_control():
    global BB_vel

    if abs(delta_pose.theta) > ANGLE_TOL:
        BB_vel.angular.z = delta_pose.theta
    else:
        BB_vel.angular.z = 0

    if abs(np.sqrt(delta_pose.x**2 + delta_pose.y**2)) > DISTANCE_TOL:
        BB_vel.linear.x = abs(np.sqrt(delta_pose.x**2 + delta_pose.y**2))
    else:
        BB_vel.linear.x = 0

#callbacks
def handleOdo_BB(msg):
    global BB_pose
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    BB_pose.x = msg.pose.pose.position.x
    BB_pose.y = msg.pose.pose.position.y
    BB_pose.theta = yaw

    global TARGET_LIST
    global current_target

    for i in range(len(TARGET_LIST)):

        if np.sqrt(delta_pose.x**2 + delta_pose.y**2) < DISTANCE_TOL:

            current_target.x = TARGET_LIST[i][0]
            current_target.y = TARGET_LIST[i][1]
        print(current_target)

#node
def burgerbot_driver():
    #init node
    rospy.init_node("burgerbot_driver", anonymous = False)
    rate = rospy.Rate(5)
    #subscribers
    rospy.Subscriber(TS_BURGERBOT_ODOM, Odometry, handleOdo_BB)
    #publishers
    vel_pub = rospy.Publisher(TP_BB_CMDVEL, Twist, queue_size=10)
    while not rospy.is_shutdown():
        try:

            calculate_delta_pose()
            motion_control()

            vel_pub.publish(BB_vel)
        except:
            pass
        rate.sleep()

    rospy.spin()

if __name__== '__main__':
    burgerbot_driver()
