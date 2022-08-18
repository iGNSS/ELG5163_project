#!/usr/bin/env python

#package imports
import numpy as np
import rospy

#message imports
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

#subscribed topics
TS_QC_ODO  = "/quadcopter/ground_truth/state"
T_TGT_VS = "/target_coords"
#published topics
TP_QC_VEL = "/quadcopter/cmd_vel"

#global varialbles
copter_pose = Odometry()
vel = Twist()

#constants
TARGET_POS = Pose()
TARGET_POS.position.x = 1
TARGET_POS.position.y = 0
TARGET_POS.position.z = 3.5
POS_TOL = 0.1

#functions
def handleOdo_QC(msg):
    global copter_pose
    copter_pose = msg

def xy_speed_control():
    global vel

def z_speed_control():
    global vel

#subscribe to camera pixel xy pos and size r






#main loop
def quadcopter_driver():
    #init node and subscribers

    rospy.init_node('quadcopter_driver', anonymous = False)
    rate = rospy.Rate(60)
    rospy.Subscriber(TS_QC_ODO , Odometry, handleOdo_QC)

    #publishers
    vel_pub = rospy.Publisher(TP_QC_VEL, Twist, queue_size=20)


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
