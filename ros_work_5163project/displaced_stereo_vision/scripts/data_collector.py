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
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

newdir = os.path.dirname(os.path.dirname(__file__)) + "/data"
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
BURGERBOT_DISTANCE_WB_LIDAR_SUB_TOPIC = "/wafflebot/lidar_distance"
SIMULATION_RUNNING_SUB_TOPIC = "/burgerbot/final_target"


###################   Class   #################################################

class DataCollector:

    def __init__(self):
        
        rospy.init_node('data_collector', anonymous=False)

        while not int(rospy.get_time()):
            print("Timer not ready")
            rospy.sleep(0.5)
        self.start = rospy.get_time()
        
        print("Timer start at: " + str(self.start))

        self.wb2bb_dist1 = Pose()
        self.wb2bb_dist2 = Pose()
        self.lidar_dist = Pose()
        self.sim_run = False

        self.wb2bb_dist_1_sub = rospy.Subscriber(BURGERBOT_DISTANCE_WB_1_SUB_TOPIC,Pose,self.update_dist_1_data)
        self.wb2bb_dist_2_sub = rospy.Subscriber(BURGERBOT_DISTANCE_WB_2_SUB_TOPIC,Pose,self.update_dist_2_data)
        self.wb2bb_dist_lidar_sub = rospy.Subscriber(BURGERBOT_DISTANCE_WB_LIDAR_SUB_TOPIC,Pose,self.update_dist_lidar_data)
        self.sim_run_sub = rospy.Subscriber(SIMULATION_RUNNING_SUB_TOPIC,Bool,self.update_sim_state)

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

    def update_sim_state(self, rec_msg):
        self.sim_run = rec_msg.data
        print("Checking if simulation ended.")

def main(args):
    
    dc = DataCollector()
    r = rospy.Rate(2) # Rate of 2 Hz
    while not rospy.is_shutdown():
        if dc.sim_run:
            rospy.signal_shutdown("Simulation is over.")
        r.sleep()
    
if __name__ == '__main__':
    main(sys.argv)
