#!/usr/bin/env python
from operator import truediv
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
import random
class robot:
    priority = False
    goal_in_sight = False
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

      
    def go_for_the_goal(self,F,mask3,w):
        # The final dash that tunnel visions on the goal, dashing straight for it using modified 'line follower' code.
        if F['m00'] == 0:
            print("Mission complete.")
            
        F = cv2.moments(mask3)
        gx = int(F['m10']/F['m00'])
        gy = int(F['m01']/F['m00'])
            
        err = gx - w/2
        self.twist.linear.x = 0.3
        self.twist.angular.z = -float(err) / 100
        self.pub.publish(self.twist)
        self.priority = True
        
    def scan_and_search(self,laser_msg):
        # Code from https://github.com/LCAS/teaching/blob/13e519fd645970c61e54f5bc3d1ec524bb9cfd9a/cmp3103m-code-fragments/scripts/lecture_02.py
        a = len(laser_msg.ranges)
        # 0:106, 106:212,  212:320
        if laser_msg.ranges[a/3] < 0.5:
            print("Oh no! A wall!")
            self.priority = True
            self.twist.linear.x = -0.2
            self.twist.angular.z= -0.2
            self.pub.publish(self.twist)
        elif laser_msg.ranges[(a/3*2):-1] < 0.5:
            print("Oh no! A wall!")
            self.priority = True
            self.twist.linear.x = -0.2
            self.twist.angular.z= 0.2
            self.pub.publish(self.twist)
        else:
            self.priority = False

    def image_callback(self, data):
        #namedWindow("Image window")
        
        cv2.namedWindow("Image window", 1)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image3 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #hsv = cv_image
        lower_yellow = numpy.array([0, 100, 100])
        upper_yellow = numpy.array([0, 255, 255])
        
        lower_red = numpy.array([0, 0, 0])
        upper_red = numpy.array([10, 50, 255])

        lower_green = numpy.array([0, 100, 0])
        upper_green = numpy.array([0, 255, 0])
       
        lower_blue = numpy.array([100, 0, 0])
        upper_blue = numpy.array([255, 0, 0])
        mask = cv2.inRange(cv_image, lower_yellow, upper_yellow)
        mask2 = cv2.inRange(cv_image2,lower_red, upper_red)
        mask3 = cv2.inRange(cv_image3,lower_green, upper_green)
        mask4 = cv2.inRange(cv_image,lower_blue, upper_blue)

        h, w, d = cv_image.shape
        search_top = h/15
        search_bot = search_top + 20
        #For testing masks
        cv_image2[h/4*3:,0:-1] = 0 
        
        mask[0:h, 0:w/5] = 0
        mask[0:h, (w/5)*4:-1] = 0
        
        mask2[h/4*3:, 0:-1] = 0


        if(self.goal_in_sight == True):
            try:
                F = cv2.moments(mask3)
                self.go_for_the_goal(F,mask3,w)
            except:
                print("whoops")
        else:
            # Taken from https://github.com/LCAS/teaching/blob/lcas_melodic/ros_book_line_follower/src/follower_p.py

            M = cv2.moments(mask)
            M2 = cv2.moments(mask4)
            # Bascially the same as the yellow line, but reverse engineered to run AWAY from red squares
            F = cv2.moments(mask2)
            if F['m00'] > 0:
                gx = int(F['m10']/F['m00'])
                gy = int(F['m01']/F['m00'])
                
                err = gx - w/2
                self.twist.linear.x = 0.1
                self.twist.angular.z = float(err) / 80
                self.pub.publish(self.twist)
                self.priority = True
            else:
                self.priority = False
            # And now again but for green. This one, however, will mindlessly dash for the green tile
            
            F = cv2.moments(mask3)
            if F['m00'] > 0:
                self.goal_in_sight = True
            if self.priority == False:
                # Focus on homing on in the yellow line
                if M['m00'] > 0:
                    #count = 0

                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    
                    cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
                    err = cx - w/2
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = -float(err) / 100
                    if M2['m00'] > 0:
                        cx = int(M2['m10']/M2['m00'])
                        cy = int(M2['m01']/M2['m00'])
                        
                        cv2.circle(cv_image, (cx, cy), 20, (0, 0, 255), -1)
                        err = cx - w/2
                        self.twist.linear.x = 0.1
                        self.twist.angular.z = -float(err) / 2000
                    self.pub.publish(self.twist)
                else: # If a yellow line cannot be found, rotate on the spot until one is found
                    
                        #random.seed(self.twist.linear.x)
                        a = random.randint(1,10)
                        if(a < 5):
                            print("spinning")
                            self.twist.angular.z = 0.6
                            self.pub.publish(self.twist)
                        else:
                            print("spinning aaaa")
                            self.twist.angular.z = -0.6
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