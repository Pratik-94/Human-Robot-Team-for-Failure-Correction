#! /usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(msg):
    #ros msg to cv2 converter object
    bridge = CvBridge()

    #Convert the ROS Msg to cv2
    image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    
    #Show the image
    cv2.imshow("Image",image)

    cv2.waitKey(3)

def main():
    #Create the node
    rospy.init_node('scarapulator',anonymous=True)

    #Create the subscriber
    sub = rospy.Subscriber('/camera/color/image_raw',Image,callback,queue_size=1)

    #Subscriber loop
    rospy.spin()

if __name__ == '__main__':
    main()
