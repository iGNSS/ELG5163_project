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

cameraModel = PinholeCameraModel()

# from urdf
HT_quad_cam = np.array([[0, 0, -1, 0],
                        [0, 1, 0, 0],
                        [-1, 0, 0, -0.3],
                        [0, 0, 0, 1]])
#from quadcopter/front_cam/camera/camera_info
FOC_LEN = 184.751
Proj_mat = np.array([[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,0],
                     [0,0,1/FOC_LEN, 0]])
#finds the center of red pixels
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

#takes in quaternion (qw, qx, qy, qz) and translation vector (x,y,z)
#returns homogenous transformation
def HT_from_odom(Q,TV):
        # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    result = np.array([[r00, r01, r02, TV[0]],
                       [r10, r11, r12, TV[1]],
                       [r20, r21, r22, TV[2]],
                        [0,0,0,1]])

    return result

class image_converter:
  camera_params = CameraInfo()
  camera_pos = Pose()
  camera_transform = np.array([[1,0,0,0],
                              [0,1,0,0],
                              [0,0,1,0],
                              [0,0,0,1]])
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_1",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(T_IMG_SUB,Image,self.callback)
    self.cam_info_sub = rospy.Subscriber(T_CAM_INFO,CameraInfo, self.get_camera_info)
    self.cam_pose = rospy.Subscriber(T_COPTER_POSE, Odometry, self.get_copter_pose)


  def get_camera_info(self,data):
      self.camera_params = data

  def get_copter_pose(self,data):
      copter_pose = data
      #print(data)
      orientation_q = data.pose.pose.orientation
      quat = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
      trans_vect = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]

      myHT = HT_from_odom(quat, trans_vect)
      self.camera_transform = HT_quad_cam @ myHT


  def find_wafflebot(self):
      cameraModel.fromCameraInfo(self.camera_params)
      cameraModel.width = 640
      cameraModel.height = 480

      point = (0,1,0)
      #camera_point = cameraModel.project3dToPixel(point)
      #print(camera_point)
      red_center = find_reds(cv_image)
      #print(red_center)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      try:
          burgerbot_coords = find_reds(cv_image)
          print(burgerbot_coords)

          (rows,cols,channels) = cv_image.shape
          if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (int(burgerbot_coords[1]),int(burgerbot_coords[0])), 30, 180)
      except:
          pass
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      #print(self.camera_params)
      print(self.camera_transform)
      self.find_wafflebot()
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('quadcopter_camera', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
