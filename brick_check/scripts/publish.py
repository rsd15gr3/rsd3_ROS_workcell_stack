#!/usr/bin/env python
import roslib
roslib.load_manifest('brick_check')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from check_brick.srv import *

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("/usb_cam/image_raw",Image,queue_size=1)

    self.bridge = CvBridge()

  def run(self):
    try:
      cv_image = cv2.imread("/home/moro/ros_workcell/src/rsd3_ROS_workcell_stack/brick_check/res/red.png")
    except CvBridgeError as e:
      print(e)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  rate = rospy.Rate(10)
  try:
    while not rospy.is_shutdown():
        ic.run()
        rate.sleep()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
