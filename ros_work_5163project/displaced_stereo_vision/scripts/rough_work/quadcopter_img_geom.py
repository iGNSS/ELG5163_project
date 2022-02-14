#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('displaced_stereo_vision')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from image_geometry import PinholeCameraModel

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
#change topic for overhead camera
#need to modify launch file
T_IMG_SUB = "/quadcopter/front_cam/camera/image"
T_CAM_INFO = "quadcopter/front_cam/camera/camera_info"
T_COPTER_POSE = "/quadcopter/ground_truth/state"
T_WAFFLEBOT_POSE = "/wafflebot/odom"
def find_red_center(img):
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = mask0+mask1
    red_pixels = np.argwhere(mask>0)
    ypx = red_pixels[:,0]
    xpx = red_pixels[:,1]
    red_center = [np.mean(ypx), np.mean(xpx)]
    return red_center

class image_converter:
    camera_params = CameraInfo()
    cameraModel = PinholeCameraModel()
    wafflebotPose = Odometry()
    burgerbot_coords = [0,0]
    burgerbot_vector = (0,0,0)
    wafflebot_coords = (0,0)
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_1",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(T_IMG_SUB, Image, self.callback)
        self.cam_info_sub = rospy.Subscriber(T_CAM_INFO, CameraInfo, self.get_camera_info)
        self.wafflebot_pos_sub = rospy.Subscriber(T_WAFFLEBOT_POSE, Odometry, self.get_wafflebot_coords)

    def get_camera_info(self,data):
        self.camera_params = data
        self.cameraModel.fromCameraInfo(data)

    def get_wafflebot_coords(self, data):
        self.wafflebotPose = data

    def get_burgerbot_vector(self):
        #print(self.cameraModel.K)

        input_coords = (self.burgerbot_coords[0], self.burgerbot_coords[1])
        #print(input_coords)
        burgerbot_vector = self.cameraModel.projectPixelTo3dRay(input_coords)
        #print(burgerbot_vector)

    def wafflebot_img_coords(self):
        waffle_x = self.wafflebotPose.pose.pose.position.x
        waffle_y = self.wafflebotPose.pose.pose.position.y
        waffle_z = self.wafflebotPose.pose.pose.position.z
        input_coords = (waffle_x, waffle_y, waffle_z)
        output_coords = self.cameraModel.project3dToPixel(input_coords)
        print(output_coords)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
            try:
                self.burgerbot_coords = find_red_center(cv_image)
                self.get_burgerbot_vector()
                self.wafflebot_img_coords()
                (rows,cols,channels) = cv_image.shape
                if cols > 60 and rows > 60 :
                    cv2.circle(cv_image, (int(burgerbot_coords[1]),int(burgerbot_coords[0])), 30, 180)

            except:
                pass
        except CvBridgeError as e:
            print(e)

        try:
          #print(self.camera_params)
          #print(self.camera_params)

          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        except CvBridgeError as e:
          print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('quadcopter_img_geom', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
