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
TS_QC_ODO = "/quadcopter/ground_truth/state"
TS_WB_ODO = "/wafflebot/odom"
TS_QC_BBDIR = "/quadcopter/front_cam/rgb/burgerbot_dir"
TS_WB_BBDIR = "/wafflebot/camera/rgb/burgerbot_dir"
#published topics
TP_BURGERBOT_DIST = "/burgerbot_distance"

#global variables
QC_Odo = Odometry()
WB_Odo = Odometry()
qc2bb = Pose()
wb2bb = Pose()
burgetbot_distance = Float32()
#constants
#HT quadcopter to quadcopter cam
#HT wafflebot to wafflebot cam

#callback functions

#need to work with odometry
def handleOdo_QC(msg):
    global QC_Odo
    QC_Odo = msg

def handleOdo_WB(msg):
    global WB_Odo
    WB_Odo = msg

def handle_qc2bb(msg):
    global qc2bb
    qc2bb = msg

def handle_wb2bb(msg):
    global wb2bb
    wb2bb = msg

def line3D(pose_vect, k):
    position = np.array([pose_vect.position.x, pose_vect.position.y, pose_vect.position.z])
    orientation_vector = np.array([pose_vect.orientation.x, pose_vect.orientation.y, pose_vect.orientation.z])
    result = position + k*orientation_vector
    return result

#functions
def calculate_location():
    global qc2bb
    global wb2bb
    a1 = (qc2bb.orientation.x)**2 + (qc2bb.orientation.y)**2 + (qc2bb.orientation.z)**2
    b1 = -(qc2bb.orientation.x)*(wb2bb.orientation.x) - (qc2bb.orientation.y)*(wb2bb.orientation.y) - (qc2bb.orientation.z)*(wb2bb.orientation.z)

    c1 = (qc2bb.position.x - wb2bb.position.x)*qc2bb.orientation.x + (qc2bb.position.y - wb2bb.position.y)*qc2bb.orientation.y +(qc2bb.position.z - wb2bb.position.z)*qc2bb.orientation.z

    a2 = -b1
    b2 = -(wb2bb.orientation.x)**2 - (wb2bb.orientation.y)**2 - (wb2bb.orientation.z)**2
    c2 = (qc2bb.position.x - wb2bb.position.x)*wb2bb.orientation.x + (qc2bb.position.y - wb2bb.position.y)*wb2bb.orientation.y + (qc2bb.position.z - wb2bb.position.z)*wb2bb.orientation.z


    A_mat = np.array([[a1, b1],
                      [a2, b2]])

    B_mat = np.array([[-c1],
                      [-c2]])

    closest_point_idx = np.linalg.inv(A_mat) @ B_mat
    k = closest_point_idx[0][0]
    s = closest_point_idx[1][0]

    qc2bb_point = line3D(qc2bb, k)
    wb2bb_point = line3D(wb2bb, s)
    #result = 0.5*(qc2bb_point + wb2bb_point) - np.array([wb2bb.orientation.x, wb2bb.orientation.y,wb2bb.orientation.z])
    result = 0.5*(wb2bb_point+qc2bb_point)
    return(result)

def calculate_distance():
    burgerbot_location = calculate_location()
    wafflebot_location = np.array([wb2bb.position.x, wb2bb.position.y, wb2bb.position.z])
    displacement = burgerbot_location - wafflebot_location
    result = np.sqrt(displacement[0]**2 + displacement[1]**2 + displacement[2]**2)
    return result


#node
def distance_calculator():
    #init node
    rospy.init_node("distance_calculator", anonymous = False)
    rate = rospy.Rate(60)
    #subscribers
    rospy.Subscriber(TS_QC_ODO, Odometry, handleOdo_QC)
    rospy.Subscriber(TS_WB_ODO, Odometry, handleOdo_WB)
    rospy.Subscriber(TS_QC_BBDIR, Pose, handle_qc2bb)
    rospy.Subscriber(TS_WB_BBDIR, Pose, handle_wb2bb)

    #publishers
    distance_pub = rospy.Publisher(TP_BURGERBOT_DIST, Float32, queue_size = 10)

    while not rospy.is_shutdown():

        try:
            visual_distance_vector = calculate_location()
            print(visual_distance_vector)
            bb_distance = calculate_distance()
            distance_pub.publish(bb_distance)

        except:
            pass
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    distance_calculator()
