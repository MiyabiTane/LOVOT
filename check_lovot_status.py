#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import copy
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import subprocess

LAMP_PATH = "images/lamp.png"
HORN_PATH = "images/horn.png"
NEST_PATH = "images/nest.png"

class CheckStatus:
    def __init__(self):
        self.bridge = CvBridge()
        self.img = None
        self.nest_img = cv2.imread(NEST_PATH)
        self.lamp_img = cv2.imread(LAMP_PATH)
        self.horn_img = cv2.imread(HORN_PATH)
        self.TH = 0.6
        self.status = -1
        self.tm = time.time()

        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_cb)


    def send_mail(self, num):
        if num == 0:
            subprocess.call("echo LOVOTの充電がしばらく行われていません。ネストに戻れていない可能性があるので73B2付近の方はLOVOTをネストに戻してあげてください。 | mail -s LOVOTの状態 -r tanemoto.tba29@gmail.com tanemoto@jsk.imi.i.u-tokyo.ac.jp", shell=True)
        elif num == 1:
            subprocess.call("echo LOVOTがネスト付近にいますが充電できていません。ネストとLOVOTの充電端子が接続されているか確かめて下さい。LOVOTの充電がまだ残っている場合は、ステイモードやお着替えモードになっていないことを確認した上でネスト付近に置き直してあげてください。 | mail -s LOVOTの状態 -r tanemoto.tba29@gmail.com tanemoto@jsk.imi.i.u-tokyo.ac.jp", shell=True)
        # 確認用
        elif num == 2:
            subprocess.call("echo LOVOTが充電中です。 | mail -s LOVOTの状態 -r tanemoto.tba29@gmail.com tanemoto@jsk.imi.i.u-tokyo.ac.jp", shell=True)


    def search_range(self, input_img):
        H, W, _C = self.nest_img.shape
        res = cv2.matchTemplate(input_img, self.nest_img, cv2.TM_CCOEFF_NORMED)
        _min_val, _max_val, _min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        # bottom_right = (top_left[0] + W, top_left[1] + H)
        # cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), thickness=2, lineType=cv2.LINE_4)

        monitor_lt = (top_left[0] + 5, top_left[1] + H - 10)
        monitor_rb = (monitor_lt[0] + 65, monitor_lt[1] + 60)
        monitor_img = self.img[monitor_lt[1]: monitor_rb[1], monitor_lt[0]: monitor_rb[0], :]
        # cv2.rectangle(img, monitor_lt, monitor_rb, (0, 255, 0), thickness=2, lineType=cv2.LINE_4)

        panel_lt = (top_left[0] - 90, top_left[1] - 20)
        panel_rb = (panel_lt[0] + 100, panel_lt[1] + 60)
        panel_img = self.img[panel_lt[1]: panel_rb[1], panel_lt[0]: panel_rb[0], :]
        # cv2.rectangle(img, panel_lt, panel_rb, (0, 0, 255), thickness=2, lineType=cv2.LINE_4)

        return monitor_img, panel_img


    def check_status(self, monitor_img, panel_img):
        """
        return integer
        0 : LOVOTが充電中
        1 : LOVOTがネスト付近にいるが充電できていない
        2 : LOVOTは充電中でない
        3 : LOVOTとネストの接続が切れている
        """
        horn_status = False
        lamp_status = False

        res1 = cv2.matchTemplate(panel_img, self.horn_img, cv2.TM_CCOEFF_NORMED)
        loc1 = np.where(res1 > self.TH)
        if len(loc1) > 5:
            horn_status = True

        res2 = cv2.matchTemplate(monitor_img, self.lamp_img, cv2.TM_CCOEFF_NORMED)
        loc2 = np.where(res2 > self.TH)
        if len(loc2) > 5:
            lamp_status = True

        if not horn_status:  # LOVOTが充電場所にいない
            if not lamp_status:
                return 2
            else:
                return 3
        else:  # LOVOTが充電場所にいる
            if not lamp_status:
                return 1
            else:
                return 0


    def img_cb(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        monitor_img, panel_img = self.search_range(self.img)
        status = self.check_status(monitor_img, panel_img)
        print(status)


if __name__ == '__main__':
    rospy.init_node("LOVOT")
    check = CheckStatus()
    rospy.spin()