#!/usr/bin/env python
#from __future__ import print_function

#package imports
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

#declare topic names
TS_QC_IMG =  "/quadcopter/front_cam/camera/image"

#published topics
TP_QC_I = "/quadcopter/front_cam/rgb/pub_img"
TP_BURGERBOT_PX_QC = "/quadcopter/front_cam/rgb/burgerbot_px"

#define functions
def find_black_center(img):
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # lower mask (0-30)
    lower_black1 = np.array([0,0,0])
    upper_black1 = np.array([30,30,30])
    mask0 = cv2.inRange(img_hsv, lower_black1 , upper_black1)

    # upper mask (170-180)
    lower_black2 = np.array([30,30,30])
    upper_black2 = np.array([50,50,50])
    mask1 = cv2.inRange(img_hsv, lower_black2, upper_black2)

    # join my masks
    mask = mask0+mask1

    black_pixels = np.argwhere(mask>0)

    ypx = np.mean(black_pixels[:,0])
    xpx = np.mean(black_pixels[:,1])
    result = (ypx, xpx)
    return result

def find_closest_red(img):
    #find center of black pixels
    ypx, xpx = find_black_center(img)

    #find all red pixels
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = mask0+mask1
    red_px = np.argwhere(mask>0)

    #find red pixel closest to black center
    x_disp = red_px[:,0]-ypx
    y_disp = red_px[:,1]-xpx
    all_disp = np.sqrt(x_disp**2 + y_disp**2)
    index_min = np.argmin(all_disp)
    closest_red = red_px[index_min]
    closest_red_coord = (closest_red[0], closest_red[1])
    return closest_red_coord

#node
class image_converter:
    burgerbot_coords = [0,0]
    burgerbot_px = Pose()
    def __init__(self):
        #important inits
        self.bridge = CvBridge()

        #publishers
        self.image_pub = rospy.Publisher(TP_QC_I, Image, queue_size=10)
        self.burgerbot_px_pub = rospy.Publisher(TP_BURGERBOT_PX_QC, Pose, queue_size=10)

        #subscribers
        self.image_sub = rospy.Subscriber(TS_QC_IMG, Image, self.callback)

    def burgerbot_px_pose(self):
        self.burgerbot_px.position.x = self.burgerbot_coords[0]
        self.burgerbot_px.position.y = self.burgerbot_coords[1]

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            cv2.imshow("Image window", cv_image)
            cv2.waitKey(3)
        except CvBridgeError as e:
            print(e)
        try:

          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
          self.burgerbot_coords = find_closest_red(cv_image)
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
