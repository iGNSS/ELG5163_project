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

#target positions in x,y coordinates
#TARGET_LIST =[[3,0.75], [3.5,-1.5]]
TARGET_LIST =[[3,1],
               [4,0],
               [3,-1],
               [2,0]]
current_target = Pose2D()
current_target.x = TARGET_LIST[0][0]
current_target.y = TARGET_LIST[0][1]

t_i = 0
delta_pose = Pose2D()
BB_pose = Pose2D()
BB_vel = Twist()
At_final = Bool()

#constants
DISTANCE_TOL = 0.12
ANGLE_TOL = 0.05


def motion_control():
    global BB_vel
    
    target_distance = np.hypot(delta_pose.x, delta_pose.y)    

    if target_distance>DISTANCE_TOL:
        
        BB_vel.linear.x = 0.2*(np.cos(delta_pose.theta))
        BB_vel.angular.z = 0.7*(np.sin(delta_pose.theta))
        
    else:
        BB_vel.linear.x = 0.0
        BB_vel.angular.z = 0.0
  


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

    #If orientation error not between -pi and pi
    if abs(delta_pose.theta) > np.pi:
        if delta_pose.theta > 0:
            delta_pose.theta -= (2 * np.pi)
        else:
            delta_pose.theta += (2 * np.pi)

    global TARGET_LIST
    global At_final

    # for i in range(len(TARGET_LIST)):
    #
    #     if np.sqrt(delta_pose.x**2 + delta_pose.y**2) < DISTANCE_TOL:
    #         print(i)
    #         current_target.x = TARGET_LIST[i][0]
    #         current_target.y = TARGET_LIST[i][1]

    global t_i
    if np.hypot(delta_pose.x, delta_pose.y) < DISTANCE_TOL:
        t_i = t_i+1
        try:
            current_target.x = TARGET_LIST[t_i][0]
            current_target.y = TARGET_LIST[t_i][1]
            At_final.data = False
        except:
            At_final.data = True
            print("at final target")

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

            motion_control()

            vel_pub.publish(BB_vel)
            bool_pub.publish(At_final)
        except:

            pass
        rate.sleep()

    rospy.spin()

if __name__== '__main__':
    burgerbot_driver()
