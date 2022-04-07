#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_conversions import transformations

from cv2 import namedWindow, cvtColor, imshow
from sensor_msgs.msg import Image


from cv_bridge import CvBridge
from cv2 import COLOR_BGR2GRAY, waitKey
import numpy 
import cv2
class robot:
    def __init__(self):
        # https://github.com/LCAS/teaching/blob/lcas_melodic/cmp3103m-code-fragments/scripts/opencv_bridge.py
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_callback)
        self.pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 1)
      #  self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image, self.depth_callback)
        # Move first
        # Snippets Taken from https://github.com/LCAS/teaching/blob/lcas_melodic/cmp3103m-code-fragments/scripts/move_square.py
        
        # 5 hz
        r = rospy.Rate(5)
        # Move command is a twist command, giving us control of direction
        #move_cmd = Twist()
        #move_cmd.linear.x = 0.1
        # While rospy is running, keep moving
        #while not rospy.is_shutdown():
        #    self.pub.publish(move_cmd)
            
    def image_callback(self, data):
        #namedWindow("Image window")
        
        cv2.namedWindow("Image window", 1)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        #hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv = cv_image
        lower_yellow = numpy.array([0, 100, 100])
        upper_yellow = numpy.array([0, 255, 255])
        
        lower_red = numpy.array([0, 170, 0])
        upper_red = numpy.array([0, 255, 0])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        #mask = cv2.inRange(cv_image,upper_yellow,lower_yellow)
        
        cv2.bitwise_and(cv_image, cv_image, mask=mask)
                
        imshow("Image window", mask)
        waitKey(1)

   # def depth_callback(self,data):
   #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
   #     waitKey(1)
#    def collision(self,data):
#        angle = self.odom_orientation(data.pose.pose.orientation)
#        print "angle = %f" % angle

#    def odom_orientation(self, q):
#        y, p, r = transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])
#        return y * 180 / 3.14

    #def image_proc(self, data):
        
         

print("Loaded")
rospy.init_node('robot')
robo = robot()
rospy.spin()
# Taken from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29