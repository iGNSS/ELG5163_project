#!/usr/bin/env python
#from __future__ import print_function

import roslib
roslib.load_manifest('displaced_stereo_vision')
import sys
import rospy
import cv2
import numpy as np

#message imports
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose

#subscribed topics
TS_WB_IMG = "/wafflebot/camera/rgb/image_raw"

#published topics
TP_W_I = "/wafflebot/camera/rgb/pub_img"
TP_BURGERBOT_PX_WB = "/wafflebot/camera/rgb/burgerbot_px"

#define functions
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
    red_center = (np.mean(ypx), np.mean(xpx))
    return red_center

#node
class image_converter:
    burgerbot_coords = [0,0]
    burgerbot_px = Pose()
    def __init__(self):
        #important inits
        self.bridge = CvBridge()

        #publishers
        self.image_pub = rospy.Publisher(TP_W_I, Image, queue_size=10)
        self.burgerbot_px_pub = rospy.Publisher(TP_BURGERBOT_PX_WB, Pose, queue_size=10)

        #subscribers
        self.image_sub = rospy.Subscriber(TS_WB_IMG,Image,self.callback)

    def burgerbot_px_pose(self):
        self.burgerbot_px.position.x = self.burgerbot_coords[1]
        self.burgerbot_px.position.y = self.burgerbot_coords[0]

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
          self.burgerbot_coords = find_red_center(cv_image)
          self.burgerbot_px_pose()
          print(self.burgerbot_px)
          self.burgerbot_px_pub.publish(self.burgerbot_px)

        except CvBridgeError as e:
          print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('wafflebot_camera', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
