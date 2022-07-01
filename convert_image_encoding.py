#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class image_converter(object):
    def __init__(self, topic):
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.image_pub = rospy.Publisher(topic+'_color',Image)

    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data,desired_encoding= "bgr8")
            self.yolo_img = cv2.cvtColor(self.cv_image,cv2.COLOR_BGR2RGB)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.yolo_img, "bgr8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('image_converter', anonymous=True)
    topic = "/camera_fr/image_raw"
    ic = image_converter(topic)
    time.sleep(1)
    while 1:
        img = ic.cv_image
        # resize_img = cv2.resize(img, (640,480), interpolation = cv2.INTER_AREA)
        # cv2.imshow('wtf',resize_img)
        # cv2.waitKey(1)
        time.sleep(1)

