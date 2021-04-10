#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import copy
from datetime import datetime
from copy import deepcopy

from send_mail import send_mail_main, send_mail_debug, send_mail_debug_staus

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


LAMP_PATH = "images/lamp.png"
HORN_PATH = "images/horn.png"
NEST_PATH = "images/nest.png"
SAVE_PATH = "images/view.png"

class CheckStatus:
    def __init__(self):
        self.bridge = CvBridge()
        self.img = None
        self.nest_img = cv2.imread(NEST_PATH)
        self.lamp_img = cv2.imread(LAMP_PATH)
        self.horn_img = cv2.imread(HORN_PATH)
        self.TH = 0.7
        self.status_lst = []  # 0: 充電中でない, 1: 充電中
        self.info = (datetime.now().hour, datetime.now().minute, -1)  # (statusが変わった時間, status)
        self.send_flag = False

        self.search = 0  # -1: 夜の充電中, 0: カメラとネストの間に障害物がある, 1: ネストが認識できている

        self.debug_mail_flag = -1

        self.get_up_time = rospy.get_param('~start_time', 11)
        self.go_bed_time = rospy.get_param('~end_time', 22)
        self.debug_view = rospy.get_param('~debug_view', False)
        
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_cb, queue_size=1)


    def search_range(self, input_img):
        H, W, _C = self.nest_img.shape
        res = cv2.matchTemplate(input_img, self.nest_img, cv2.TM_CCOEFF_NORMED)
        _min_val, max_val, _min_loc, max_loc = cv2.minMaxLoc(res)
        # print(max_val)
        if max_val < self.TH:
            self.search = 0
        else:
            self.search = 1
        top_left = max_loc

        monitor_lt = (top_left[0] + 5, top_left[1] + H - 10)
        monitor_rb = (monitor_lt[0] + 65, monitor_lt[1] + 60)
        monitor_img = self.img[monitor_lt[1]: monitor_rb[1], monitor_lt[0]: monitor_rb[0], :]

        panel_lt = (top_left[0] - 90, top_left[1] - 20)
        panel_rb = (panel_lt[0] + 100, panel_lt[1] + 60)
        panel_img = self.img[panel_lt[1]: panel_rb[1], panel_lt[0]: panel_rb[0], :]

        if self.debug_view:
            output_img = deepcopy(self.img)
            bottom_right = (top_left[0] + W, top_left[1] + H)
            cv2.rectangle(output_img, top_left, bottom_right, (255, 0, 0), thickness=2, lineType=cv2.LINE_4)
        
            cv2.rectangle(output_img, monitor_lt, monitor_rb, (0, 255, 0), thickness=2, lineType=cv2.LINE_4)
        
            cv2.rectangle(output_img, panel_lt, panel_rb, (0, 0, 255), thickness=2, lineType=cv2.LINE_4)
            cv2.imwrite("debug.png", output_img)

        if self.search == 1:
            horn_status, lamp_status = self.check_status(monitor_img, panel_img)

        return horn_status, lamp_status


    def check_status(self, monitor_img, panel_img):
        """
        return interger, ineteger (horn_status, lamp_status)
        -1: None
        0: False
        1: True 
        """
        horn_status = -1
        lamp_status = -1
        try:
            res1 = cv2.matchTemplate(panel_img, self.horn_img, cv2.TM_CCOEFF_NORMED)
            _min_val, max_val_1, _min_loc, max_loc_1 = cv2.minMaxLoc(res1)
            if max_val_1 > self.TH:
                horn_status = 1
            else:
                horn_status = 0
        except:
            pass

        try:
            res2 = cv2.matchTemplate(monitor_img, self.lamp_img, cv2.TM_CCOEFF_NORMED)
            _min_val, max_val_2, _min_loc, max_loc_2 = cv2.minMaxLoc(res2)
            if max_val_2 > self.TH:
                lamp_status = 1
            else:
                lamp_status = 0
        except:
            pass
        # print(max_val_1, max_val_2)

        if self.debug_view:
            H, W, _C = self.horn_img.shape
            top_left = max_loc_1
            bottom_right = (top_left[0] + W, top_left[1] + H)
            cv2.rectangle(panel_img, top_left, bottom_right, (255, 0, 0), thickness=2, lineType=cv2.LINE_4)
            cv2.imwrite("debug_horn.png", panel_img)

            H, W, _C = self.lamp_img.shape
            top_left = max_loc_2
            bottom_right = (top_left[0] + W, top_left[1] + H)
            cv2.rectangle(monitor_img, top_left, bottom_right, (255, 0, 0), thickness=2, lineType=cv2.LINE_4)
            cv2.imwrite("debug_lamp.png", monitor_img)

        return horn_status, lamp_status


    def calc(self, cur_hour, cur_minute, prev_hour, prev_minute):
        if cur_minute < prev_minute:
            cur_hour -= 1
            cur_minute += 60
        diff = (cur_minute - prev_minute) + (cur_hour - prev_hour) * 60
        # print(cur_hour, cur_minute, prev_hour, prev_minute, diff)
        return diff

    def img_cb(self, msg):
        """
        (horn_status, lamp_status) = (1, 1) : LOVOTが充電中
        else : LOVOTは充電中でない
        """
        cur_time = datetime.now()
        cur_hour = cur_time.hour
        cur_minute = cur_time.minute
        self.img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.search == -1 and cur_hour == self.get_up_time:
            self.search = 0
            cv2.imwrite(SAVE_PATH, self.img)
            send_mail_main(2, SAVE_PATH)
        if cur_hour == self.go_bed_time and cur_minute == 30:
            self.search = -1
        if self.search >= 0:
            # デバッグ用
            if (cur_minute == 0 or cur_minute == 30) and self.debug_mail_flag != cur_minute:
                send_mail_debug_staus(self.info)
                self.debug_mail_flag = cur_minute
            horn_status, lamp_status = self.search_range(self.img)
            if horn_status == -1 or lamp_status == -1:
                status = -1
            elif horn_status == 1 and lamp_status == 1:
                status = 1
            else:
                status = 0
            """
            -1: 判定不可
            0: LOVOTが充電中でない
            1: LOVOTが充電中
            """
            if status != -1:
                self.status_lst.append(status)
            if len(self.status_lst) == 60:
                state_len = len(set(self.status_lst))
                if state_len == 1:  # ノイズがない情報
                    keep_hour, keep_minute, keep_status = self.info
                    if keep_status != status:
                        self.info = (cur_hour, cur_minute, status)
                        self.send_flag = False
                    if keep_status == 0 and status == 1:
                        cv2.imwrite(SAVE_PATH, self.img)
                        send_mail_debug(SAVE_PATH)
                    elif keep_status == 0 and status == 0 and not self.send_flag:
                        if self.calc(cur_hour, cur_minute, keep_hour, keep_minute) >= 60:  # 1時間以上充電されていない
                            cv2.imwrite(SAVE_PATH, self.img)
                            if horn_status:
                                send_mail_main(1, SAVE_PATH)
                            else:
                                send_mail_main(0, SAVE_PATH)
                            self.send_flag = True
                self.status_lst = []
            # print(self.info)


if __name__ == '__main__':
    rospy.init_node("LOVOT")
    check = CheckStatus()
    rospy.spin()
