#!/usr/bin/env python

#package imports
import numpy as np
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

#message imports
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

#subscribed topics
TS_WB_LIDAR = "wafflebot/scan"

#published topics
TP_BURGERBOT_DIST = "wafflebot/lidar_data"

#global variables
sensed_object = Pose2D()

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
    #init node, declare subscriber, publisher and rate


    rospy.init_node('wafflebot_lidar', anonymous=False)
    rospy.Subscriber(TS_WB_LIDAR , LaserScan, detect_object)
    pub = rospy.Publisher(TP_BURGERBOT_DIST, Pose2D, queue_size=10)
    rate = rospy.Rate(20)
    listener = tf.TransformListener()

    while not rospy.is_shutdown():

        #publish sensed_object
        pub.publish(sensed_object)

        #log sensed_object
        dist_string = "distance to object(m): " + str(sensed_object.x)
        angle_string = "angle to object(degrees): " + str(sensed_object.theta)
        logstring ="\n" +dist_string + "\n" + angle_string  +"\n"

        try:
            (transLid2Cam, rotLid2Cam) = listener.lookupTransform(LIDAR_FRAME , CAMERA_FRAME, rospy.Time(0))
            # transLid2Cam = transRot[0]
            # rotLid2Cam = transRot[1]
            HTLid2Cam = listener.fromTranslationRotation(transLid2Cam, rotLid2Cam)
            print(HTLid2Cam)

        except:
            pass

        rospy.loginfo(logstring)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':

        laser_scan_reader()
