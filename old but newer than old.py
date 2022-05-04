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
        
        self.twist = Twist()
      #  self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image, self.depth_callback)
        # Move first
        # Snippets Taken from https://github.com/LCAS/teaching/blob/lcas_melodic/cmp3103m-code-fragments/scripts/move_square.py
        
        # 5 hz
        r = rospy.Rate(5)
        
        # Move command is a twist command, giving us control of direction
        move_cmd = Twist()
        #move_cmd.linear.x = 0.1
        # While rospy is running, keep moving
        #while not rospy.is_shutdown():
        #    self.pub.publish(move_cmd)
            
    def image_callback(self, data):
        #namedWindow("Image window")
        
        cv2.namedWindow("Image window", 1)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #hsv = cv_image
        lower_yellow = numpy.array([0, 100, 100])
        upper_yellow = numpy.array([0, 255, 255])
        
        lower_red = numpy.array([0, 170, 0])
        upper_red = numpy.array([0, 255, 0])
        mask = cv2.inRange(cv_image, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(cv_image2,upper_red,lower_red)
                
        #cv2.bitwise_and(cv_image, cv_image, mask=mask)
        #cv2.bitwise_and(cv_image2,cv_image2,mask=mask2)
        h, w, d = cv_image.shape
        search_top = h/15
        search_bot = search_top + 20
        
        mask[0:search_top, 0:w] = 0
        #mask[search_bot:h, 0:w] = 0
        
        # Taken from https://github.com/LCAS/teaching/blob/lcas_melodic/ros_book_line_follower/src/follower_p.py

        M = cv2.moments(mask)
        # Focus on homing on in the yellow line
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            print self.twist.angular.z
            self.pub.publish(self.twist)
        else: # If a yellow line cannot be found, rotate on the spot until one is found, favouring a left turn 
            self.twist.angular.z= 1
            #self.twist.linear.x = 0.2
            self.pub.publish(self.twist)
        imshow("Image window", mask)
        waitKey(1)


    # Step one, find wall, follow wall.
    # Step two, check for blue squares, if a blue square is spotted, override step one and move to the blue sqaure
    # Step three, detect lava, avoid it at all costs, turn until an opening is spotted and return to step one

    #def image_proc(self, data):
        
         

print("Loaded")
rospy.init_node('robot')
robo = robot()
rospy.spin()
# Taken from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29