#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

from cv2 import namedWindow, cvtColor, imshow
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv2 import COLOR_BGR2GRAY, waitKey
class robot:
    def __init__(self):
        # https://github.com/LCAS/teaching/blob/lcas_melodic/cmp3103m-code-fragments/scripts/opencv_bridge.py
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.image_callback)
        self.pub = rospy.Publisher("/mobile_base/commands/velocity",Twist,queue_size = 1)
        
        # Move first
        # Snippets Taken from https://github.com/LCAS/teaching/blob/lcas_melodic/cmp3103m-code-fragments/scripts/move_square.py
        
        r = rospy.Rate(5)
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        while not rospy.is_shutdown():
            self.pub.publish(move_cmd)
    def image_callback(self, data):
        namedWindow("Image window")
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray_img = cvtColor(cv_image, COLOR_BGR2GRAY)
        imshow("Image window", cv_image)
        
        waitKey(1)
       
print("Loaded")
rospy.init_node('robot')
robo = robot()
rospy.spin()
# Taken from http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29