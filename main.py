#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_conversions import transformations

from sensor_msgs.msg import LaserScan

from cv2 import namedWindow, cvtColor, imshow
from sensor_msgs.msg import Image


from cv_bridge import CvBridge
from cv2 import COLOR_BGR2GRAY, waitKey
import numpy 
import cv2
class robot:
    priority = False
    def __init__(self):
        # https://github.com/LCAS/teaching/blob/lcas_melodic/cmp3103m-code-fragments/scripts/opencv_bridge.py
        self.bridge = CvBridge()
        self.laser_sub = rospy.Subscriber("/scan", LaserScan,self.scan_and_search)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.image_callback)
        self.pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 1)
        
        self.twist = Twist()
        # self.depth_sub = rospy.Subscriber("/camera/depth/image_raw",Image, self.depth_callback)
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
        cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        #gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #hsv = cv_image
        lower_yellow = numpy.array([0, 100, 100])
        upper_yellow = numpy.array([0, 255, 255])
        
        #lower_red = numpy.array([0, 170, 0])
        #upper_red = numpy.array([0, 255, 0])
        lower_red = numpy.array([0, 0, 0])
        upper_red = numpy.array([10, 50, 255])
       
        mask = cv2.inRange(cv_image, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(cv_image2,lower_red,upper_red)
                
        #cv2.bitwise_and(cv_image, cv_image, mask=mask)
        #cv2.bitwise_and(cv_image2,cv_image2,mask=mask2)
        h, w, d = cv_image.shape
        search_top = h/15
        search_bot = search_top + 20
        
        #mask[0:search_top, 0:w] = 0
        mask[0:h, 0:w/5] = 0
        mask[0:h, (w/5)*4:-1] = 0
        mask2[-1:(h/5)*4, 0:w] = 0
        #mask[0:h, 0+(w/20):w/5] = 0
        #mask[0:h, (w/5)*4:w-(w/20)] = 0
        #mask[0:h, 0:w/5] = 0
        #mask[0:h, (w/5)*4:w] = 0
        
        #mask[search_bot:h, 0:w] = 0
        
        # Taken from https://github.com/LCAS/teaching/blob/lcas_melodic/ros_book_line_follower/src/follower_p.py

        M = cv2.moments(mask)
        # Bascially the same as the yellow line, but reverse engineered to run AWAY from red squares
        F = cv2.moments(mask2)
        # Focus on homing on in the yellow line
        if F['m00'] > 0:
            gx = int(F['m10']/F['m00'])
            gy = int(F['m01']/F['m00'])
            
            err = gx - w/2
            self.twist.linear.x = 0.1
            self.twist.angular.z = float(err) / 100
            self.pub.publish(self.twist)
            self.priority = True
        else:
            self.priority = False
        if M['m00'] > 0:
            #count = 0
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
            if self.priority == False:
                err = cx - w/2
                self.twist.linear.x = 0.1
                self.twist.angular.z = -float(err) / 100
                self.pub.publish(self.twist)
        else: # If a yellow line cannot be found, rotate on the spot until one is found, favouring a left turn 
            if self.priority == False:
                self.twist.linear.x = -0.1
                self.pub.publish(self.twist)
            #count = count + 0.1
            #if(count > 90):
            #self.twist.linear.x = 0
            #self.twist.angular.z= -0.1
            #else:
            #    self.twist.linear.x = 0
            #    self.twist.angular.z= 0.1
            #self.twist.linear.x = 0.2 # Make it check left and right?
            #self.pub.publish(self.twist)
        imshow("Image window", mask2)
        waitKey(1)

    def scan_and_search(self,laser_msg):
        # Code from https://github.com/LCAS/teaching/blob/13e519fd645970c61e54f5bc3d1ec524bb9cfd9a/cmp3103m-code-fragments/scripts/lecture_02.py
        a = len(laser_msg.ranges)
        # 0:106, 106:212,  212:320
        if laser_msg.ranges[a/3] < 1.0:
            self.priority = True
            self.twist.linear.x = 0
            self.twist.angular.z= -0.5
            self.pub.publish(self.twist)
        #elif laser_msg.ranges[a/3:(a/3)*2] < 0.1:
            #self.priority = True
            #self.twist.linear.x = -0.3
            #self.pub.publish(self.twist)
        elif laser_msg.ranges[(a/3*2):-1] < 1.0:
            self.priority = True
            self.twist.linear.x = 0
            self.twist.angular.z= 0.5
            self.pub.publish(self.twist)
        else:
            self.priority = False
        #    t = Twist()
        #    t.linear.x = 1.0
        #    self.pub.publish(t)

    # Step one, find wall, follow wall.
    # Step two, check for blue squares, if a blue square is spotted, override step one and move to the blue sqaure
    # Step three, detect lava, avoid it at all costs, turn until an opening is spotted and return to step one

    #def image_proc(self, data):
        
         

print("Loaded")
rospy.init_node('robot')
robo = robot()
rospy.spin()
# Taken from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29