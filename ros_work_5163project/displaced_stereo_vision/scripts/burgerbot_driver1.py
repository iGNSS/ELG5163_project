#!/usr/bin/env python

#package imports
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#message imports
from geometry_msgs.msg import Pose, Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

#subscribed topics
TS_BURGERBOT_ODOM = "/burgerbot/odom"

#publish topics
TP_BB_CMDVEL = "/burgerbot/cmd_vel"
TP_BB_AT_FINAL = "burgerbot/final_target"
#global variables
TARGET_LIST =[[4,1], [5,0]]

current_target = Pose2D()
current_target.x = TARGET_LIST[0][0]
current_target.y = TARGET_LIST[0][1]
delta_pose = Pose2D()
BB_pose = Pose2D()
BB_vel = Twist()
At_final = Bool()

#constants
DISTANCE_TOL = 0.12
ANGLE_TOL = 0.05

def calculate_delta_pose():
    global delta_pose
    delta_pose.x = current_target.x - BB_pose.x
    delta_pose.y = current_target.y - BB_pose.y
    target_angle = np.arctan2(delta_pose.y, delta_pose.x)
    delta_pose.theta = target_angle - BB_pose.theta

def theta_close_enough():
    global delta_pose
    if abs((delta_pose.theta))<ANGLE_TOL:
        output = True
    else:
        output = False

    return output

def position_close_enough():
    global delta_pose
    if np.sqrt(delta_pose.x**2+delta_pose.y**2) > DISTANCE_TOL:
        output = False
    else:
        output = True

    return output


def motion_control():
    global BB_vel
    target_distance = np.sqrt(delta_pose.x**2 + delta_pose.y**2)

    if abs(delta_pose.theta)>ANGLE_TOL and target_distance>DISTANCE_TOL:
        BB_vel.linear.x = 0
        if abs(delta_pose.theta)>2*ANGLE_TOL  and target_distance>DISTANCE_TOL:
            BB_vel.linear.x = 0
            BB_vel.angular.z = delta_pose.theta
        if abs(delta_pose.theta)<2*ANGLE_TOL and abs(delta_pose.theta)>ANGLE_TOL and target_distance>DISTANCE_TOL:
            BB_vel.linear.x = 0
            BB_vel.angular.z = 0.25*abs(delta_pose.theta)/delta_pose.theta +ANGLE_TOL/2


    if abs(delta_pose.theta)<ANGLE_TOL and target_distance>DISTANCE_TOL:
        BB_vel.angular.z = 0
        BB_vel.linear.x = target_distance

    if target_distance<DISTANCE_TOL:
        BB_vel.angular.z = 0
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

    global delta_pose
    global current_target
    delta_pose.x = current_target.x - BB_pose.x
    delta_pose.y = current_target.y - BB_pose.y
    target_angle = np.arctan2(delta_pose.y, delta_pose.x)
    delta_pose.theta = target_angle - BB_pose.theta

    global TARGET_LIST
    global At_final

    for i in range(len(TARGET_LIST)):
        if np.sqrt(delta_pose.x**2 + delta_pose.y**2) < DISTANCE_TOL:
            current_target.x = TARGET_LIST[i][0]
            current_target.y = TARGET_LIST[i][1]
        

#node
def burgerbot_driver():
    #init node
    rospy.init_node("burgerbot_driver", anonymous = False)
    rate = rospy.Rate(20)
    #subscribers
    rospy.Subscriber(TS_BURGERBOT_ODOM, Odometry, handleOdo_BB)
    #publishers
    vel_pub = rospy.Publisher(TP_BB_CMDVEL, Twist, queue_size=10)
    bool_pub = rospy.Publisher(TP_BB_AT_FINAL, Bool, queue_size=10)
    while not rospy.is_shutdown():
        try:

            #calculate_delta_pose()
            #move_to_target()
            motion_control()
            print(current_target)

            print(BB_vel)
            vel_pub.publish(BB_vel)
        except:
            pass
        rate.sleep()

    rospy.spin()

if __name__== '__main__':
    burgerbot_driver()
