#! /usr/bin/env python3
from __future__ import print_function
from re import X

# import roslib
# roslib.load_manifest('enph353_ros_lab')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

CUT_SIZE = 600
EMA_WEIGHT = 0.4
THRESHOLD = 120
RADIUS = 15


class image_converter:

  def __init__(self):
    self.twist_pub = rospy.Publisher("cmd_vel",Twist)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("robot/camera1/image_raw",Image,self.callback)
    self.x_ema = 0
    self.y_ema = 0
    self.cx = 0
    self.cy = 0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Crop image
    cropped_im = cv_image[CUT_SIZE:,:]
    # Clean image + mask
    blur = cv2.GaussianBlur(cropped_im,(5,5),0)
    _, mask = cv2.threshold(blur, THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    gray = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
    # Get countour
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_color = (0, 255, 0)
    contour_thick = 10
    with_contours = cv2.drawContours(gray, contours, -1, contour_color, contour_thick)
    # Determine contour centroid
    if len(contours) > 0:
        cnt = contours[0]
        M = cv2.moments(cnt)
        self.cx = int((M['m10']+0.00001)/(M['m00']+0.00001))
        self.cy = int((M['m01']+0.00001)/(M['m00']+0.00001))
    else:
        pass
   
    # Calculate exponential moving average EMA
    x_pos = self.cx
    y_pos = self.cy + CUT_SIZE
    if self.x_ema == 0:
        self.x_ema = x_pos
        self.y_ema = y_pos
    else: 
        self.x_ema = EMA_WEIGHT*x_pos + (1-EMA_WEIGHT)*self.x_ema
        self.y_ema = EMA_WEIGHT*y_pos + (1-EMA_WEIGHT)*self.y_ema
    # Mark Image
    gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
    color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    color = cv2.circle(color, (int(self.x_ema),int(self.y_ema)), RADIUS, (0,0,255), -1)
    cv2.imshow("Image window",color)
    cv2.waitKey(3)
    # Compute velocity and rotation
    move = Twist()
    move.linear.x = self.y_ema / (800*2)
    move.angular.z = (-5 * (self.x_ema - 400)) / 800
    try:
      self.twist_pub.publish(move)
    except CvBridgeError as e:
      print(e)
    
def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)