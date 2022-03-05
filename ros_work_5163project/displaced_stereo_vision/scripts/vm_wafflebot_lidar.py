#!/usr/bin/env python

#package imports
import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#message imports
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D, Pose
from nav_msgs.msg import Odometry

#subscribed topics
TS_WB_LIDAR = "wafflebot/scan"

#published topics
TP_BURGERBOT_DIST = "wafflebot/lidar_distance"

#global variables
sensed_object = Pose2D()
burgerbot_distance = Pose()
#constants
LIDAR_FRAME = 'wafflebot_tf/base_scan'
CAMERA_FRAME = "wafflebot_tf/camera_rgb_optical_frame"

#functions
# saves data from LIDAR into sensed_object
def detect_object(msg):

    #check if object exists
    #
    try:
        ranges = msg.ranges

        # convert to numpy array to be able to use numpy functions
        npranges = np.array(ranges)

        # convert values out of range to 'NaN' to be ignored in calculation
        npranges[npranges > msg.range_max] = float('NaN')
        npranges[npranges < msg.range_min] = float('NaN')

        # compute minimum distance and its corresponding angles with respect to scanner's frame
        min_distance = np.nanmin(npranges)
        indices = np.reshape( np.argwhere(npranges == min_distance) , -1)

        angle_to_target = float(((indices*msg.angle_increment)+msg.angle_min))
        distance_to_target =  min_distance

        global sensed_object
        sensed_object.x = min_distance
        sensed_object.theta = angle_to_target
        #if no object then nan value for distance and angle
    except:
        sensed_object.x = np.nan
        sensed_object.theta = np.nan

#function to be executed when running this node
def laser_scan_reader():
    global burgerbot_distance
    #init node,
    rospy.init_node('wafflebot_lidar', anonymous=False)
    rate = rospy.Rate(5)
    listener = tf.TransformListener()
    #subscribers
    rospy.Subscriber(TS_WB_LIDAR , LaserScan, detect_object)
    #publishers
    distance_pub = rospy.Publisher(TP_BURGERBOT_DIST, Pose, queue_size = 10)

    while not rospy.is_shutdown():

        HTvect = np.array([[sensed_object.x*np.cos(sensed_object.theta)],
                                 [sensed_object.x*np.sin(sensed_object.theta)],
                                [0],
                                [1]])

        try:
            (transLid2Cam, rotLid2Cam) = listener.lookupTransform(LIDAR_FRAME , CAMERA_FRAME, rospy.Time(0))

            HTLid2Cam = listener.fromTranslationRotation(transLid2Cam, rotLid2Cam)
            HTobj_cam = np.matmul(np.linalg.inv(HTLid2Cam), HTvect)
            HTdistance = np.sqrt(HTobj_cam[0][0]**2 + HTobj_cam[1][0]**2 + HTobj_cam[2][0]**2)
            burgerbot_distance.position.x = HTdistance
            distance_pub.publish(burgerbot_distance)
            print(burgerbot_distance)

        except:
            pass

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':

        laser_scan_reader()
