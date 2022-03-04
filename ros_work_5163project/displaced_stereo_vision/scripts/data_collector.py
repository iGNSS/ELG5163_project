#!/usr/bin/env python
"""
This code is part of the displaced_stereo_vision project for ELG 5163 Machine
Vision.

The objective of this node is to collect the distance from three topics and save
the data in csv files.

Created on Feb 11 2022

@author: Mathieu Falardeau
"""

###################    main library imports    ################################
from __future__ import print_function

import roslib
roslib.load_manifest('displaced_stereo_vision')
import sys
import os
import rospy
from geometry_msgs.msg import Pose, Pose2D

newdir = os.path.dirname(os.getcwd()) + "/data"
print(newdir)
try:
    os.mkdir(newdir)
except OSError:
    rospy.loginfo("Directory " + newdir + " already exists!!")

DISTANCE_DATA_COLLECTOR_1_FILEPATH = newdir + "/dist_calc_1_data.csv"
DISTANCE_DATA_COLLECTOR_2_FILEPATH = newdir + "/dist_calc_2_data.csv"
DISTANCE_DATA_COLLECTOR_LIDAR_FILEPATH = newdir + "/dist_lidar_data.csv"

BURGERBOT_DISTANCE_WB_1_SUB_TOPIC = "/burgerbot_distance"
BURGERBOT_DISTANCE_WB_2_SUB_TOPIC = "/wafflebot/camera/rgb/burgerbot_distance"
BURGERBOT_DISTANCE_WB_LIDAR_SUB_TOPIC = "/burgerbot_distance_lidar"


###################   Class   #################################################

class DataCollector:

    def __init__(self):
        
        rospy.init_node('data_collector', anonymous=False)

        while not int(rospy.get_time()):
            print("Timer not ready")
        self.start = rospy.get_time()
        
        print("Timer start at: " + str(self.start))

        self.wb2bb_dist1 = Pose()
        self.wb2bb_dist2 = Pose()
        self.lidar_dist = Pose2D()

        self.wb2bb_dist_1_sub = rospy.Subscriber(BURGERBOT_DISTANCE_WB_1_SUB_TOPIC,Pose,self.update_dist_1_data)
        self.wb2bb_dist_2_sub = rospy.Subscriber(BURGERBOT_DISTANCE_WB_2_SUB_TOPIC,Pose,self.update_dist_2_data)
        self.wb2bb_dist_lidar_sub = rospy.Subscriber(BURGERBOT_DISTANCE_WB_LIDAR_SUB_TOPIC,Pose2D,self.update_dist_lidar_data)

        header = 'Time(sec),Distance(m)'

        filepaths =[DISTANCE_DATA_COLLECTOR_1_FILEPATH,DISTANCE_DATA_COLLECTOR_2_FILEPATH,DISTANCE_DATA_COLLECTOR_LIDAR_FILEPATH] 

        for fp in filepaths:
            with open(fp,'w') as f:
                f.write(header)

    def update_dist_1_data(self, rec_msg):
        self.wb2bb_dist1 = rec_msg
        self.update_data(rospy.get_time(),self.wb2bb_dist1.position.x,DISTANCE_DATA_COLLECTOR_1_FILEPATH)
        print("Writing distance 1")
        

    def update_dist_2_data(self, rec_msg):
        self.wb2bb_dist2 = rec_msg
        self.update_data(rospy.get_time(),self.wb2bb_dist2.position.x,DISTANCE_DATA_COLLECTOR_2_FILEPATH)
        print("Writing distance 2")

    def update_dist_lidar_data(self, rec_msg):
        self.lidar_dist = rec_msg
        self.update_data(rospy.get_time(),self.lidar_dist.position.x,DISTANCE_DATA_COLLECTOR_LIDAR_FILEPATH)
        print("Writing distance lidar")

    def update_data(self, now_time, dist,filepath):
        
        duration = now_time - self.start
        
        t = str(duration)
        d = str(dist)
                
        with open(filepath,'a') as f:
            f.write('\n'+t+','+d)

def main(args):
    
    dc = DataCollector()
    rospy.spin()
    

if __name__ == '__main__':
    main(sys.argv)
