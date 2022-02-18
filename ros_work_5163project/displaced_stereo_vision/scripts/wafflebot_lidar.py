#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

sensed_object = Pose2D()
TP_BURGERBOT_DIST_L = "/burgerbot_distance_lidar"
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

        angle_to_target = float(((indices*msg.angle_increment)+msg.angle_min)*180.0/np.pi)
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
    rospy.Subscriber('wafflebot/scan', LaserScan, detect_object)
    pub = rospy.Publisher('lidar_distance', Pose2D, queue_size=10)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        #publish sensed_object
        pub.publish(sensed_object)

        #log sensed_object
        dist_string = "distance to object(m): " + str(sensed_object.x)
        angle_string = "angle to object(degrees): " + str(sensed_object.theta)
        logstring ="\n" +dist_string + "\n" + angle_string  +"\n"
        rospy.loginfo(logstring)

        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    laser_scan_reader()
