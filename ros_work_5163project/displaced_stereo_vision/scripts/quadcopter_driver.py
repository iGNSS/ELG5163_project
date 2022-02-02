#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#Topics
T_COPTER_POSE = "/quadcopter/ground_truth/state"
T_COPTER_VEL = "/quadcopter/cmd_vel"

#global varialbles
copter_pose = Odometry()
vel = Twist()

#constants
TARGET_POS = Pose()
TARGET_POS.position.x = 1
TARGET_POS.position.y = 0
TARGET_POS.position.z = 5
POS_TOL = 0.1

#functions
def get_copter_pose(msg):
    global copter_pose
    copter_pose = msg

def speed_control():
    global vel
    copter_z = copter_pose.pose.pose.position.z
    copter_x = copter_pose.pose.pose.position.x
    if (TARGET_POS.position.z - copter_z) > POS_TOL:
        vel.linear.z = (TARGET_POS.position.z - copter_z)
    else:
        vel.linear.z = 0

    if (TARGET_POS.position.x - copter_x) > POS_TOL:
        vel.linear.x = (TARGET_POS.position.x - copter_x)
    else:
        vel.linear.x = 0

def print_pos():
    print("x = " + str(copter_pose.pose.pose.position.x))
    print("y = " + str(copter_pose.pose.pose.position.y))
    print("z = " + str(copter_pose.pose.pose.position.z))
    print("")
    
#main loop
def quadcopter_driver():
    #init node and subscribers

    rospy.init_node('quadcopter_driver', anonymous = False)
    rate = rospy.Rate(60)
    rospy.Subscriber(T_COPTER_POSE, Odometry, get_copter_pose)

    #publishers
    vel_pub = rospy.Publisher(T_COPTER_VEL, Twist, queue_size=20)


    while not rospy.is_shutdown():

        speed_control()
        vel_pub.publish(vel)
        print_pos()
        rate.sleep()

    rospy.spin()

if __name__ =='__main__':
    try:
        quadcopter_driver()
    except rospy.ROSInterruptException:
        pass
