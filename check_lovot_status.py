#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
# from std_msgs.msg import Int16
from cv_bridge import CvBridge

LAMP_YES_PATH = "images/lamp_yes.png"
LAMP_NO_PATH = "images/lamp_no.png"
HORN_YES_PATH = "images/horn_yes.png"
HORN_NO_PATH = "images/horn_no.png"

class CheckStatus:
    def __init__(self):
        self.bridge = CvBridge()
        self.img = None
        self.TH = 0.6
        self.status = 1
        # self.pub_msg = Int16()

        # self.pub = rospy.Publisher("/lovot_status", Int16, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_cb)


    def check_yes_no(self, input_img, yes_img, no_img):
        # use template matching
        # if match to "yes", return True
        match_result_no = cv2.matchTemplate(input_img, no_img, cv2.TM_CCOEFF_NORMED)
        match_result_yes = cv2.matchTemplate(input_img, yes_img, cv2.TM_CCOEFF_NORMED)
        loc1 = np.where(match_result_no >= self.TH)
        loc2 = np.where(match_result_yes >= self.TH)
        if len(loc2[0]) > len(loc1[0]):
            return True
        return False


    def img_cb(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        lamp_img = self.img[200:250, 400:460, :]
        horn_img = self.img[120:180, 300:400, :]
        ltemp_yes_img = cv2.imread(LAMP_YES_PATH)
        ltemp_no_img = cv2.imread(LAMP_NO_PATH)
        htemp_yes_img = cv2.imread(HORN_YES_PATH)
        htemp_no_img = cv2.imread(HORN_NO_PATH)
        lamp_status = self.check_yes_no(lamp_img, ltemp_yes_img, ltemp_no_img)
        horn_status = self.check_yes_no(horn_img, htemp_yes_img, htemp_no_img)
        if (not lamp_status) and horn_status:
            self.status = 2  # LOVOTがネスト付近にいるが充電できていない
        elif (not lamp_status) and (not horn_status):
            self.status = 0  # LOVOTが充電中でない
        elif lamp_status and horn_status:
            self.status = 1  # LOVOTがネストで充電中
        print(self.status)


    """
    def debug_view(self, loc):
        min_y = min(loc[0])
        max_y = max(loc[0])
        min_x = min(loc[1])
        max_x = max(loc[1])
        debug_img = self.img[240 + min_y: 240 + ]
    """

if __name__ == '__main__':
    rospy.init_node("LOVOT")
    check = CheckStatus()
    rospy.spin()