#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('displaced_stereo_vision')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
IMAGE_SUB_TOPIC = "/wafflebot/camera/rgb/image_raw"

def find_reds(img):
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

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(IMAGE_SUB_TOPIC,Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      try:
          burgerbot_coords = find_reds(cv_image)
          print(burgerbot_coords)
      except:
          pass

      #
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
